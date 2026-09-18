#include "topics.hpp"
#include <common/srv/get_color.hpp>

enum RobotColor {
    COLOR_INVALID,
    COLOR_RED,
    COLOR_BLUE
};

RobotColor GetRobotColor(const std::string &name)
{
    if (name.find("red") != std::string::npos) {
        return COLOR_RED;
    } else if (name.find("blue") != std::string::npos) {
        return COLOR_BLUE;
    } else {
        return COLOR_INVALID;
    }
}

int GetRobotId(const std::string &name)
{
    return static_cast<int>(name.back() - '0');
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    std::string robotName = "maxwell";
    std::string friendName = "maxwell";

    std::string tmp = std::string(argv[1]);
    std::string team = tmp.substr(0, tmp.find_last_of('_'));

    auto playerNode = std::make_shared<rclcpp::Node>(tmp + "_player");
    rclcpp::Client<common::srv::GetColor>::SharedPtr client =
        playerNode->create_client<common::srv::GetColor>("gamectrl/get_color");
    while (!client->wait_for_service(std::chrono::duration<long long>(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(playerNode->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(playerNode->get_logger(), "service not available, waiting again...");
    }
    auto request = std::make_shared<common::srv::GetColor::Request>();
    request.get()->team = team;
    auto result = client->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(playerNode, result);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        auto resp = result.get();
        if (resp->color == "invalid") {
            RCLCPP_ERROR(playerNode->get_logger(), "Not supportted team name. Exiting.");
            return 0;
        }
        robotName = resp->color + tmp.substr(tmp.find_last_of('_'));
        friendName = resp->color;
        RCLCPP_INFO(playerNode->get_logger(), "robotName: %s", robotName.c_str());
    } else {
        RCLCPP_ERROR(playerNode->get_logger(), "Exiting.");
        return 0;
    }

    RobotColor myColor = GetRobotColor(robotName);
    int myId = GetRobotId(robotName);
    friendName = friendName + "_" + std::to_string(3 - myId);
    common::msg::BodyTask btask;
    common::msg::HeadTask htask;
    common::msg::GameData gameData;
    common::msg::Location location;
    btask.type = btask.TASK_WALK;
    btask.count = 2;
    btask.step = 0.03;
    htask.yaw = 0.0;
    htask.pitch = 45.0;
    auto bodyTaskNode = std::make_shared<BodyTaskPublisher>(robotName);
    auto headTaskNode = std::make_shared<HeadTaskPublisher>(robotName);
    auto imageSubscriber = std::make_shared<ImageSubscriber>(robotName);
    auto imuSubscriber = std::make_shared<ImuDataSubscriber>(robotName);
    auto headSubscriber = std::make_shared<HeadAngleSubscriber>(robotName);
    auto resImgPublisher = std::make_shared<ResultImagePublisher>(robotName);
    auto gameSubscriber = std::make_shared<GameDataSubscriber>(robotName);
    auto locSubscriber = std::make_shared<LocationSubscriber>(robotName);
    rclcpp::WallRate loop_rate(10.0);
    float initYaw = 0.0;

    while (rclcpp::ok()) {
        rclcpp::spin_some(bodyTaskNode);
        rclcpp::spin_some(headTaskNode);
        rclcpp::spin_some(imageSubscriber);
        rclcpp::spin_some(imuSubscriber);
        rclcpp::spin_some(headSubscriber);
        rclcpp::spin_some(resImgPublisher);
        auto imuData = imuSubscriber->GetData();
        auto image = imageSubscriber->GetImage().clone();
        auto headAngle = headSubscriber->GetData();

        rclcpp::spin_some(gameSubscriber);
        gameData = gameSubscriber->GetData();

        rclcpp::spin_some(locSubscriber); // 更新定位
        location = locSubscriber->GetData();

        // ----------------- 可以修改的部分 begin--------------------
        // V1 + V2 + V3：完全软编码的足球视觉 / 自动搜索 / 自主追球验证版。
        // 当前测试终点是“自主追到近球安全区并停止”，尚不接球门瞄准、脚前定位和踢球。
        // 设计原则：
        // 1) 足球位置只来自相机，不使用 Webots/Supervisor 真值；location 仅显示调试信息。
        // 2) 远距离允许低增益连续视觉追踪；进入近距离后严格执行“运动 -> 停稳 -> 静止重捕获”。
        // 3) 足球非白色部分颜色随机，因此检测不再依赖黑色花纹，而依赖圆形、白色主体、
        //    内部非绿色、外围绿色、圆周边缘以及时间连续性。
        // 4) 所有动作时序继续按真正收到的新相机帧推进，不使用墙钟时间估算 Webots 动作进度。

        enum class SoftPhase {
            WAIT_PLAY,
            SEARCH_BALL,
            SEARCH_BODY_TURN,
            SEARCH_BODY_SETTLE,
            FACE_BALL_TURN,
            FACE_BALL_SETTLE,
            CHASE_REACQUIRE,
            CHASE_FAR,
            NEAR_SETTLE,
            NEAR_REACQUIRE,
            NEAR_TURN,
            NEAR_TURN_SETTLE,
            NEAR_FORWARD,
            NEAR_FORWARD_SETTLE,
            V3_NEAR_READY_STOP,
            RECOVER
        };

        struct SearchPose {
            double yaw;
            double pitch;
        };

        struct BallObservation {
            bool found = false;
            cv::Point center;
            int radius = 0;
            double score = 0.0;
            double whiteRatio = 0.0;
            double greenInsideRatio = 0.0;
            double greenRingRatio = 0.0;
            double edgeRatio = 0.0;
        };

        // 头部搜索先覆盖远球常见低俯仰，再覆盖近球俯视。
        // 搜索姿态只是感知动作，不包含任何“球应该在哪”的位置先验。
        static const std::vector<SearchPose> searchPoses = {
            {0.0, 20.0},
            {-35.0, 20.0},
            {35.0, 20.0},
            {-70.0, 20.0},
            {70.0, 20.0},
            {0.0, 42.0},
            {-35.0, 42.0},
            {35.0, 42.0},
            {-70.0, 42.0},
            {70.0, 42.0}
        };

        // 近球重捕获优先保持身体正前方，只在丢失时小范围改变头姿。
        static const std::vector<SearchPose> nearSearchPoses = {
            {0.0, 35.0},
            {0.0, 45.0},
            {0.0, 55.0},
            {15.0, 45.0},
            {-15.0, 45.0}
        };

        // ---------- 相机 / 几何 ----------
        static constexpr double cameraHorizontalFovRad = 1.3613;
        static constexpr double radToDeg = 57.29577951308232;

        // ---------- 通用时序 ----------
        static constexpr int motionCommandFrames = 15;
        static constexpr int turnSettleFrames = 18;
        static constexpr int walkSettleFrames = 22;
        static constexpr int headSearchMissFramesPerPose = 7;
        static constexpr int staticBallConfirmFrames = 3;
        static constexpr int chaseLostFrameLimit = 8;
        static constexpr int chaseHoldFrameLimit = 4;
        static constexpr int farVisionStride = 2;

        // ---------- SEARCH_BALL ----------
        // 全头扫失败后，身体只转一个小角度再重新搜索，不依赖固定开局方向。
        static constexpr double searchBodyTurnDegrees = 24.0;
        static constexpr int searchBodyTurnCycleCount = 2;

        // ---------- CHASE_FAR ----------
        // step=0.05 是历史实测 walk.conf 上限。视觉只给小幅转向，不做剧烈追框。
        static constexpr double farStepFast = 0.05;
        static constexpr double farStepMedium = 0.04;
        static constexpr double farStepSlow = 0.03;
        static constexpr int farWalkCycleCount = 2;
        static constexpr double farTurnGain = 0.28;
        static constexpr double farTurnDeadbandDegrees = 1.5;
        static constexpr double farTurnMaxPerCycle = 2.5;
        static constexpr double liveTrackAlpha = 0.25;
        static constexpr int farToNearRadiusPixels = 30;
        static constexpr int farToNearConfirmFrames = 3;

        // ---------- CHASE_NEAR ----------
        // 历史 T6a 已实测 step=0.015,count=2 安全且可观察，因此作为近球离散前进初值。
        static constexpr double nearForwardStep = 0.015;
        static constexpr int nearForwardCycleCount = 2;
        static constexpr int maxNearForwardPulses = 8;
        static constexpr double nearTurnDeadbandDegrees = 3.0;
        static constexpr double nearTurnMaxPerCycle = 4.0;
        static constexpr int nearTurnCycleCount = 2;
        // 历史 260 帧链在 radius≈40 时仍处于约 0.5m 的安全预备区。
        // V3 暂以 40px 作为“到达近球安全区”的停止阈值，后续 V4/V5 再重新标定。
        static constexpr int v3NearReadyRadiusPixels = 40;

        // ---------- 足球检测门槛 ----------
        // 静止阶段允许较低得分；运动阶段要求更高置信度，降低模糊画面误追踪风险。
        static constexpr double staticBallMinScore = 3.6;
        static constexpr double movingBallMinScore = 4.5;

        static SoftPhase phase = SoftPhase::WAIT_PLAY;
        static int previousGameState = -1;
        static int phaseFrames = 0;
        static double activeTurnCommand = 0.0;
        static int activeTurnCycleCount = 2;
        static double headTargetYaw = 0.0;
        static double headTargetPitch = 20.0;
        static std::size_t searchPoseIndex = 0;
        static int searchPoseMissFrames = 0;
        static std::size_t nearSearchPoseIndex = 0;
        static int nearSearchPoseMissFrames = 0;

        // 静止确认器：SEARCH / REACQUIRE / NEAR 都复用，但每次状态切换都会清空。
        static int staticStableFrames = 0;
        static cv::Point staticLastCenter;
        static int staticLastRadius = 0;
        static double staticXSum = 0.0;
        static double staticYSum = 0.0;
        static double staticRadiusSum = 0.0;
        static cv::Point confirmedBallCenter;
        static int confirmedBallRadius = 0;
        static double confirmedBallScore = 0.0;

        // 运动期只维护低通后的“方向趋势”，不把单帧框直接变成大动作。
        static bool liveTrackValid = false;
        static double liveBallX = 0.0;
        static double liveBallY = 0.0;
        static double liveBallRadius = 0.0;
        static double liveBallScore = 0.0;
        static int liveLostFrames = 0;
        static int nearTriggerFrames = 0;
        static double lastFarTurnCommand = 0.0;

        static int nearForwardPulses = 0;
        static int recoverUprightFrames = 0;
        static unsigned long long totalProcessedCameraFrames = 0;

        static const auto normalizeAngle = [](double angle) {
            while (angle > 180.0) angle -= 360.0;
            while (angle <= -180.0) angle += 360.0;
            return angle;
        };

        static const auto clampValue = [](double value, double low, double high) {
            return std::max(low, std::min(high, value));
        };

        // 仿真倍率不断变化，因此状态机只按真正的新相机消息推进。
        static unsigned long long imageFrameSequence = 0;
        static auto imageFrameCounter =
            playerNode->create_subscription<sensor_msgs::msg::Image>(
                robotName + "/sensor/image",
                5,
                [](const sensor_msgs::msg::Image::SharedPtr) {
                    ++imageFrameSequence;
                }
            );
        (void)imageFrameCounter;
        rclcpp::spin_some(playerNode);

        static unsigned long long lastProcessedImageFrame = 0;
        const unsigned long long newCameraFrameCount =
            imageFrameSequence - lastProcessedImageFrame;
        const bool newCameraFrame = newCameraFrameCount > 0;
        if (newCameraFrame) {
            lastProcessedImageFrame = imageFrameSequence;
            totalProcessedCameraFrames += newCameraFrameCount;
        }
        const int motionFrameAdvance = static_cast<int>(
            std::min<unsigned long long>(newCameraFrameCount, 100ULL)
        );

        // 每个主循环都先清掉旧命令，只有当前状态明确需要动作时才重新赋值。
        btask.type = btask.TASK_WALK;
        btask.step = 0.0;
        btask.lateral = 0.0;
        btask.turn = 0.0;
        btask.count = 0;
        htask.yaw = headTargetYaw;
        htask.pitch = headTargetPitch;
        const bool robotUpright = imuData.fall == imuData.FALL_NONE;

        const auto resetStaticConfirmation = [&]() {
            staticStableFrames = 0;
            staticLastCenter = cv::Point();
            staticLastRadius = 0;
            staticXSum = 0.0;
            staticYSum = 0.0;
            staticRadiusSum = 0.0;
            confirmedBallCenter = cv::Point();
            confirmedBallRadius = 0;
            confirmedBallScore = 0.0;
        };

        const auto resetLiveTrack = [&]() {
            liveTrackValid = false;
            liveBallX = 0.0;
            liveBallY = 0.0;
            liveBallRadius = 0.0;
            liveBallScore = 0.0;
            liveLostFrames = 0;
            nearTriggerFrames = 0;
            lastFarTurnCommand = 0.0;
        };

        const auto setSearchPose = [&](const std::vector<SearchPose> &poses,
                                       std::size_t index) {
            if (index < poses.size()) {
                headTargetYaw = poses[index].yaw;
                headTargetPitch = poses[index].pitch;
                htask.yaw = headTargetYaw;
                htask.pitch = headTargetPitch;
            }
        };

        const auto enterSearchBall = [&]() {
            phase = SoftPhase::SEARCH_BALL;
            searchPoseIndex = 0;
            searchPoseMissFrames = 0;
            resetStaticConfirmation();
            resetLiveTrack();
            setSearchPose(searchPoses, searchPoseIndex);
        };

        const auto enterNearReacquire = [&]() {
            phase = SoftPhase::NEAR_REACQUIRE;
            nearSearchPoseIndex = 0;
            nearSearchPoseMissFrames = 0;
            resetStaticConfirmation();
            setSearchPose(nearSearchPoses, nearSearchPoseIndex);
        };

        // 像素横向误差 -> 相机相对身体的水平角。画面左侧为身体左转正角。
        const auto pixelToBodyAngleDegrees = [&](double pixelX) {
            if (image.empty() || image.cols <= 0) return 0.0;
            const double fx = static_cast<double>(image.cols) /
                (2.0 * std::tan(cameraHorizontalFovRad / 2.0));
            return std::atan(
                (static_cast<double>(image.cols) * 0.5 - pixelX) / fx
            ) * radToDeg;
        };

        // ---------- 新版足球检测 ----------
        // 规则只保证“白色区域 >50%，其他颜色随机”。因此这里不要求黑色花纹。
        // Hough 圆和白色轮廓两路产生候选，再统一用颜色、草地外围、圆周边缘评分。
        const auto detectBall = [&](cv::Mat &debugImage, bool drawCandidate) {
            BallObservation best;
            if (image.empty()) return best;

            cv::Mat hsv;
            cv::Mat lab;
            cv::Mat gray;
            cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV);
            cv::cvtColor(image, lab, cv::COLOR_RGB2Lab);
            cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);

            cv::Mat hsvWhite;
            cv::Mat labWhite;
            cv::Mat whiteMask;
            cv::Mat greenMask;
            cv::inRange(
                hsv,
                cv::Scalar(0, 0, 135),
                cv::Scalar(180, 105, 255),
                hsvWhite
            );
            cv::inRange(
                lab,
                cv::Scalar(135, 100, 100),
                cv::Scalar(255, 158, 158),
                labWhite
            );
            cv::bitwise_or(hsvWhite, labWhite, whiteMask);
            cv::inRange(
                hsv,
                cv::Scalar(25, 45, 25),
                cv::Scalar(95, 255, 255),
                greenMask
            );

            cv::Mat whiteForContours;
            cv::Mat whiteKernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(3, 3)
            );
            cv::morphologyEx(
                whiteMask,
                whiteForContours,
                cv::MORPH_CLOSE,
                whiteKernel
            );

            cv::Mat blurred;
            cv::GaussianBlur(gray, blurred, cv::Size(7, 7), 1.8, 1.8);
            cv::Mat edges;
            cv::Canny(blurred, edges, 45.0, 120.0);

            std::vector<cv::Vec3f> proposals;
            cv::HoughCircles(
                blurred,
                proposals,
                cv::HOUGH_GRADIENT,
                1.2,
                14.0,
                95.0,
                13.0,
                6,
                std::min(120, std::min(image.cols, image.rows) / 3)
            );

            // 轮廓候选用于补偿远球 Hough 偶发漏检；严格要求近似圆形，避免场地白线。
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(
                whiteForContours.clone(),
                contours,
                cv::RETR_EXTERNAL,
                cv::CHAIN_APPROX_SIMPLE
            );
            for (const auto &contour : contours) {
                const double area = cv::contourArea(contour);
                if (area < 20.0 || area > 12000.0) continue;
                const double perimeter = cv::arcLength(contour, true);
                if (perimeter <= 0.0) continue;
                const double circularity = 4.0 * CV_PI * area /
                    (perimeter * perimeter);
                if (circularity < 0.28) continue;

                const cv::Rect rect = cv::boundingRect(contour);
                const int longSide = std::max(rect.width, rect.height);
                const int shortSide = std::min(rect.width, rect.height);
                if (longSide <= 0 ||
                    static_cast<double>(shortSide) / longSide < 0.62) {
                    continue;
                }

                cv::Point2f centerFloat;
                float radiusFloat = 0.0f;
                cv::minEnclosingCircle(contour, centerFloat, radiusFloat);
                const int radius = cvRound(radiusFloat);
                if (radius < 5 || radius > 110) continue;
                proposals.emplace_back(
                    centerFloat.x,
                    centerFloat.y,
                    radiusFloat
                );
            }

            for (const cv::Vec3f &proposal : proposals) {
                const cv::Point center(
                    cvRound(proposal[0]),
                    cvRound(proposal[1])
                );
                const int radius = cvRound(proposal[2]);
                if (radius < 5 || radius > 120) continue;

                // 足球主体必须基本完整留在画面内；外围草地环允许部分裁切。
                if (center.x - radius < 1 || center.y - radius < 1 ||
                    center.x + radius >= image.cols - 1 ||
                    center.y + radius >= image.rows - 1) {
                    continue;
                }

                const int outerRadius = std::max(radius + 3, cvRound(radius * 1.38));
                const cv::Rect roiRect(
                    std::max(0, center.x - outerRadius),
                    std::max(0, center.y - outerRadius),
                    std::min(image.cols - std::max(0, center.x - outerRadius),
                             outerRadius * 2 + 1),
                    std::min(image.rows - std::max(0, center.y - outerRadius),
                             outerRadius * 2 + 1)
                );
                if (roiRect.width < radius * 2 || roiRect.height < radius * 2) {
                    continue;
                }

                const cv::Point localCenter = center - roiRect.tl();
                cv::Mat inner = cv::Mat::zeros(roiRect.size(), CV_8UC1);
                cv::Mat outer = cv::Mat::zeros(roiRect.size(), CV_8UC1);
                cv::Mat ringInner = cv::Mat::zeros(roiRect.size(), CV_8UC1);
                cv::Mat edgeBandOuter = cv::Mat::zeros(roiRect.size(), CV_8UC1);
                cv::Mat edgeBandInner = cv::Mat::zeros(roiRect.size(), CV_8UC1);

                cv::circle(
                    inner,
                    localCenter,
                    std::max(1, cvRound(radius * 0.90)),
                    cv::Scalar(255),
                    -1
                );
                cv::circle(
                    outer,
                    localCenter,
                    std::max(radius + 2, outerRadius),
                    cv::Scalar(255),
                    -1
                );
                cv::circle(
                    ringInner,
                    localCenter,
                    std::max(1, cvRound(radius * 1.06)),
                    cv::Scalar(255),
                    -1
                );
                cv::circle(
                    edgeBandOuter,
                    localCenter,
                    std::max(2, cvRound(radius * 1.08)),
                    cv::Scalar(255),
                    -1
                );
                cv::circle(
                    edgeBandInner,
                    localCenter,
                    std::max(1, cvRound(radius * 0.82)),
                    cv::Scalar(255),
                    -1
                );

                cv::Mat ring;
                cv::Mat edgeBand;
                cv::subtract(outer, ringInner, ring);
                cv::subtract(edgeBandOuter, edgeBandInner, edgeBand);

                const int innerPixels = cv::countNonZero(inner);
                const int ringPixels = cv::countNonZero(ring);
                const int edgeBandPixels = cv::countNonZero(edgeBand);
                if (innerPixels <= 0 || ringPixels <= 0 || edgeBandPixels <= 0) {
                    continue;
                }

                cv::Mat sampled;
                cv::bitwise_and(whiteMask(roiRect), inner, sampled);
                const double whiteRatio =
                    static_cast<double>(cv::countNonZero(sampled)) /
                    static_cast<double>(innerPixels);

                cv::bitwise_and(greenMask(roiRect), inner, sampled);
                const double greenInsideRatio =
                    static_cast<double>(cv::countNonZero(sampled)) /
                    static_cast<double>(innerPixels);

                cv::bitwise_and(greenMask(roiRect), ring, sampled);
                const double greenRingRatio =
                    static_cast<double>(cv::countNonZero(sampled)) /
                    static_cast<double>(ringPixels);

                cv::bitwise_and(edges(roiRect), edgeBand, sampled);
                const double edgeRatio =
                    static_cast<double>(cv::countNonZero(sampled)) /
                    static_cast<double>(edgeBandPixels);

                // 宽松门槛适配光照与随机彩色面片；绿色主体和缺少圆周边缘的候选被拒绝。
                if (whiteRatio < 0.20 ||
                    greenInsideRatio > 0.52 ||
                    greenRingRatio < 0.07 ||
                    edgeRatio < 0.018) {
                    continue;
                }

                const double nonGreenInsideScore =
                    1.0 - std::min(greenInsideRatio / 0.52, 1.0);
                const double score =
                    4.0 * std::min(whiteRatio / 0.60, 1.0) +
                    2.0 * nonGreenInsideScore +
                    2.5 * std::min(greenRingRatio / 0.55, 1.0) +
                    3.0 * std::min(edgeRatio / 0.16, 1.0);

                double trackBonus = 0.0;
                if (liveTrackValid && liveBallRadius > 0.0) {
                    const double dx = center.x - liveBallX;
                    const double dy = center.y - liveBallY;
                    const double distance = std::sqrt(dx * dx + dy * dy);
                    const double maxJump = std::max(45.0, 4.0 * liveBallRadius);
                    const double radiusRatio = radius /
                        std::max(1.0, liveBallRadius);
                    if (distance <= maxJump &&
                        radiusRatio >= 0.50 && radiusRatio <= 2.0) {
                        trackBonus = 1.8 * (1.0 - distance / maxJump);
                    }
                }

                const double finalScore = score + trackBonus;
                if (!best.found || finalScore > best.score) {
                    best.found = true;
                    best.center = center;
                    best.radius = radius;
                    best.score = finalScore;
                    best.whiteRatio = whiteRatio;
                    best.greenInsideRatio = greenInsideRatio;
                    best.greenRingRatio = greenRingRatio;
                    best.edgeRatio = edgeRatio;
                }
            }

            if (best.found && drawCandidate && !debugImage.empty()) {
                cv::circle(
                    debugImage,
                    best.center,
                    best.radius,
                    cv::Scalar(0, 255, 255),
                    2
                );
                cv::circle(
                    debugImage,
                    best.center,
                    3,
                    cv::Scalar(255, 0, 0),
                    -1
                );
            }
            return best;
        };

        const auto acceptStaticBall = [&](const BallObservation &observation) {
            if (!observation.found || observation.score < staticBallMinScore) {
                resetStaticConfirmation();
                return false;
            }

            bool continuous = false;
            if (staticStableFrames > 0 && staticLastRadius > 0) {
                const double dx = observation.center.x - staticLastCenter.x;
                const double dy = observation.center.y - staticLastCenter.y;
                const double distance = std::sqrt(dx * dx + dy * dy);
                const double maxJump = std::max(12.0, 0.65 * observation.radius);
                const double maxRadiusChange = std::max(5.0, 0.35 * observation.radius);
                continuous =
                    distance <= maxJump &&
                    std::abs(observation.radius - staticLastRadius) <= maxRadiusChange;
            }

            if (!continuous) {
                staticStableFrames = 1;
                staticXSum = observation.center.x;
                staticYSum = observation.center.y;
                staticRadiusSum = observation.radius;
            } else {
                ++staticStableFrames;
                staticXSum += observation.center.x;
                staticYSum += observation.center.y;
                staticRadiusSum += observation.radius;
            }

            staticLastCenter = observation.center;
            staticLastRadius = observation.radius;
            confirmedBallScore = observation.score;

            if (staticStableFrames >= staticBallConfirmFrames) {
                confirmedBallCenter = cv::Point(
                    cvRound(staticXSum / staticStableFrames),
                    cvRound(staticYSum / staticStableFrames)
                );
                confirmedBallRadius = std::max(
                    1,
                    cvRound(staticRadiusSum / staticStableFrames)
                );
                return true;
            }
            return false;
        };

        const auto seedLiveTrackFromConfirmed = [&]() {
            if (confirmedBallRadius <= 0) return;
            liveTrackValid = true;
            liveBallX = confirmedBallCenter.x;
            liveBallY = confirmedBallCenter.y;
            liveBallRadius = confirmedBallRadius;
            liveBallScore = confirmedBallScore;
            liveLostFrames = 0;
            nearTriggerFrames = 0;
            lastFarTurnCommand = 0.0;
        };

        const auto updateLiveTrack = [&](const BallObservation &observation) {
            if (!observation.found || observation.score < movingBallMinScore) {
                ++liveLostFrames;
                return false;
            }

            if (!liveTrackValid) {
                liveTrackValid = true;
                liveBallX = observation.center.x;
                liveBallY = observation.center.y;
                liveBallRadius = observation.radius;
                liveBallScore = observation.score;
                liveLostFrames = 0;
                return true;
            }

            const double dx = observation.center.x - liveBallX;
            const double dy = observation.center.y - liveBallY;
            const double distance = std::sqrt(dx * dx + dy * dy);
            const double maxJump = std::max(55.0, 4.5 * liveBallRadius);
            const double radiusRatio = observation.radius /
                std::max(1.0, liveBallRadius);
            if (distance > maxJump || radiusRatio < 0.45 || radiusRatio > 2.2) {
                ++liveLostFrames;
                return false;
            }

            liveBallX = (1.0 - liveTrackAlpha) * liveBallX +
                liveTrackAlpha * observation.center.x;
            liveBallY = (1.0 - liveTrackAlpha) * liveBallY +
                liveTrackAlpha * observation.center.y;
            liveBallRadius = (1.0 - liveTrackAlpha) * liveBallRadius +
                liveTrackAlpha * observation.radius;
            liveBallScore = observation.score;
            liveLostFrames = 0;
            return true;
        };

        // INIT 只负责彻底清空策略状态；READY/PAUSE/END 由默认零 BodyTask 停止机器人。
        if (gameData.state != previousGameState) {
            if (gameData.state == gameData.STATE_INIT) {
                phase = SoftPhase::WAIT_PLAY;
                phaseFrames = 0;
                activeTurnCommand = 0.0;
                activeTurnCycleCount = 2;
                headTargetYaw = 0.0;
                headTargetPitch = 20.0;
                searchPoseIndex = 0;
                searchPoseMissFrames = 0;
                nearSearchPoseIndex = 0;
                nearSearchPoseMissFrames = 0;
                nearForwardPulses = 0;
                recoverUprightFrames = 0;
                resetStaticConfirmation();
                resetLiveTrack();
            }
            previousGameState = gameData.state;
        }

        if (gameData.state == gameData.STATE_PLAY &&
            phase == SoftPhase::WAIT_PLAY && robotUpright) {
            enterSearchBall();
            RCLCPP_INFO(
                playerNode->get_logger(),
                "V1-V3 start: autonomous visual ball search."
            );
        }

        // 跌倒不继续推进状态机；恢复直立一段时间后从重新搜球开始。
        if (gameData.state == gameData.STATE_PLAY && !robotUpright &&
            phase != SoftPhase::RECOVER) {
            phase = SoftPhase::RECOVER;
            phaseFrames = 0;
            recoverUprightFrames = 0;
            resetStaticConfirmation();
            resetLiveTrack();
            RCLCPP_ERROR(playerNode->get_logger(), "Robot fall detected. Enter RECOVER.");
        }

        const SoftPhase phaseAtFrameStart = phase;

        cv::Mat resultImage = image.empty() ? cv::Mat() : image.clone();
        BallObservation currentObservation;
        bool currentObservationValid = false;

        // ---------- 静止搜索 / 重捕获视觉 ----------
        const bool staticVisualPhase =
            phase == SoftPhase::SEARCH_BALL ||
            phase == SoftPhase::CHASE_REACQUIRE ||
            phase == SoftPhase::NEAR_REACQUIRE;

        if (gameData.state == gameData.STATE_PLAY && robotUpright &&
            newCameraFrame && staticVisualPhase && !image.empty()) {
            const bool headAtTarget =
                std::abs(static_cast<double>(headAngle.yaw) - headTargetYaw) <= 3.0 &&
                std::abs(static_cast<double>(headAngle.pitch) - headTargetPitch) <= 3.0;

            if (headAtTarget) {
                currentObservation = detectBall(resultImage, true);
                currentObservationValid = currentObservation.found;
                const bool confirmed = acceptStaticBall(currentObservation);

                if (confirmed) {
                    if (phase == SoftPhase::SEARCH_BALL) {
                        // 搜索时头部可能偏转。身体需要先把“头部yaw + 像素偏角”消掉，
                        // 然后回到头正中再开始追球。
                        const double totalBearing = normalizeAngle(
                            static_cast<double>(headAngle.yaw) +
                            pixelToBodyAngleDegrees(confirmedBallCenter.x)
                        );
                        if (std::abs(totalBearing) <= 2.0) {
                            // 即使身体方向已经基本正确，也先把头回中并静止重捕获，
                            // 避免直接拿偏头姿态下的旧像素坐标启动连续追球。
                            phase = SoftPhase::CHASE_REACQUIRE;
                            searchPoseMissFrames = 0;
                            resetStaticConfirmation();
                            headTargetYaw = 0.0;
                            headTargetPitch = confirmedBallRadius >= farToNearRadiusPixels
                                ? 35.0 : 25.0;
                        } else {
                            activeTurnCycleCount = std::abs(totalBearing) > 45.0 ? 3 : 2;
                            activeTurnCommand = clampValue(
                                totalBearing /
                                    static_cast<double>(activeTurnCycleCount),
                                -25.0,
                                25.0
                            );
                            phase = SoftPhase::FACE_BALL_TURN;
                            phaseFrames = motionCommandFrames;
                            headTargetYaw = 0.0;
                            headTargetPitch = 25.0;
                            resetStaticConfirmation();
                        }
                    } else if (phase == SoftPhase::CHASE_REACQUIRE) {
                        seedLiveTrackFromConfirmed();
                        if (confirmedBallRadius >= farToNearRadiusPixels) {
                            phase = SoftPhase::NEAR_SETTLE;
                            phaseFrames = walkSettleFrames;
                            headTargetYaw = 0.0;
                            headTargetPitch = 35.0;
                        } else {
                            phase = SoftPhase::CHASE_FAR;
                            headTargetYaw = 0.0;
                            headTargetPitch = 25.0;
                        }
                    } else if (phase == SoftPhase::NEAR_REACQUIRE) {
                        const double totalBearing = normalizeAngle(
                            static_cast<double>(headAngle.yaw) +
                            pixelToBodyAngleDegrees(confirmedBallCenter.x)
                        );

                        if (confirmedBallRadius >= v3NearReadyRadiusPixels) {
                            phase = SoftPhase::V3_NEAR_READY_STOP;
                            RCLCPP_INFO(
                                playerNode->get_logger(),
                                "V3 PASS: near-ball safe zone reached. x=%d y=%d r=%d bearing=%.2f",
                                confirmedBallCenter.x,
                                confirmedBallCenter.y,
                                confirmedBallRadius,
                                totalBearing
                            );
                        } else if (confirmedBallRadius < farToNearRadiusPixels - 3) {
                            seedLiveTrackFromConfirmed();
                            phase = SoftPhase::CHASE_FAR;
                            headTargetYaw = 0.0;
                            headTargetPitch = 25.0;
                        } else if (std::abs(totalBearing) > nearTurnDeadbandDegrees) {
                            activeTurnCycleCount = nearTurnCycleCount;
                            activeTurnCommand = clampValue(
                                totalBearing /
                                    static_cast<double>(nearTurnCycleCount),
                                -nearTurnMaxPerCycle,
                                nearTurnMaxPerCycle
                            );
                            phase = SoftPhase::NEAR_TURN;
                            phaseFrames = motionCommandFrames;
                            headTargetYaw = 0.0;
                            headTargetPitch = 35.0;
                            resetStaticConfirmation();
                        } else if (nearForwardPulses < maxNearForwardPulses) {
                            phase = SoftPhase::NEAR_FORWARD;
                            phaseFrames = motionCommandFrames;
                            headTargetYaw = 0.0;
                            headTargetPitch = 35.0;
                            resetStaticConfirmation();
                        } else {
                            phase = SoftPhase::V3_NEAR_READY_STOP;
                            RCLCPP_ERROR(
                                playerNode->get_logger(),
                                "V3 stop: near approach pulse budget exhausted at r=%d.",
                                confirmedBallRadius
                            );
                        }
                    }
                } else {
                    if (phase == SoftPhase::SEARCH_BALL) {
                        ++searchPoseMissFrames;
                        if (searchPoseMissFrames >= headSearchMissFramesPerPose) {
                            searchPoseMissFrames = 0;
                            ++searchPoseIndex;
                            resetStaticConfirmation();
                            if (searchPoseIndex >= searchPoses.size()) {
                                activeTurnCycleCount = searchBodyTurnCycleCount;
                                activeTurnCommand =
                                    searchBodyTurnDegrees /
                                    static_cast<double>(searchBodyTurnCycleCount);
                                phase = SoftPhase::SEARCH_BODY_TURN;
                                phaseFrames = motionCommandFrames;
                                headTargetYaw = 0.0;
                                headTargetPitch = 20.0;
                            } else {
                                setSearchPose(searchPoses, searchPoseIndex);
                            }
                        }
                    } else if (phase == SoftPhase::CHASE_REACQUIRE) {
                        ++searchPoseMissFrames;
                        if (searchPoseMissFrames >= 10) {
                            enterSearchBall();
                        }
                    } else if (phase == SoftPhase::NEAR_REACQUIRE) {
                        ++nearSearchPoseMissFrames;
                        if (nearSearchPoseMissFrames >= headSearchMissFramesPerPose) {
                            nearSearchPoseMissFrames = 0;
                            ++nearSearchPoseIndex;
                            resetStaticConfirmation();
                            if (nearSearchPoseIndex >= nearSearchPoses.size()) {
                                enterSearchBall();
                            } else {
                                setSearchPose(nearSearchPoses, nearSearchPoseIndex);
                            }
                        }
                    }
                }
            }
        }

        // ---------- 远距离连续视觉追踪 ----------
        // 运动画面只用于高置信度、低通后的方向趋势；失去可靠观测时主动减速并最终停下重搜。
        if (gameData.state == gameData.STATE_PLAY && robotUpright &&
            newCameraFrame && phase == SoftPhase::CHASE_FAR && !image.empty() &&
            (totalProcessedCameraFrames % farVisionStride == 0)) {
            currentObservation = detectBall(resultImage, true);
            currentObservationValid = currentObservation.found;
            const bool liveUpdated = updateLiveTrack(currentObservation);

            if (liveUpdated && liveBallRadius >= farToNearRadiusPixels) {
                ++nearTriggerFrames;
            } else if (liveUpdated) {
                nearTriggerFrames = 0;
            }

            if (nearTriggerFrames >= farToNearConfirmFrames) {
                phase = SoftPhase::NEAR_SETTLE;
                phaseFrames = walkSettleFrames;
                headTargetYaw = 0.0;
                headTargetPitch = 35.0;
                resetStaticConfirmation();
                RCLCPP_INFO(
                    playerNode->get_logger(),
                    "Far chase -> near mode, filtered radius=%.1f",
                    liveBallRadius
                );
            } else if (liveLostFrames >= chaseLostFrameLimit) {
                enterSearchBall();
                RCLCPP_INFO(
                    playerNode->get_logger(),
                    "Far chase lost ball; return to SEARCH_BALL."
                );
            }
        }

        // ---------- 当前状态对应的运动命令 ----------
        if (gameData.state == gameData.STATE_PLAY && robotUpright) {
            switch (phase) {
            case SoftPhase::SEARCH_BODY_TURN:
            case SoftPhase::FACE_BALL_TURN:
            case SoftPhase::NEAR_TURN:
                btask.turn = activeTurnCommand;
                btask.count = activeTurnCycleCount;
                break;

            case SoftPhase::CHASE_FAR:
                if (liveTrackValid && liveLostFrames <= chaseHoldFrameLimit) {
                    const double ballBearing = pixelToBodyAngleDegrees(liveBallX);
                    double turnCommand = 0.0;
                    if (std::abs(ballBearing) > farTurnDeadbandDegrees) {
                        turnCommand = clampValue(
                            farTurnGain * ballBearing,
                            -farTurnMaxPerCycle,
                            farTurnMaxPerCycle
                        );
                    }
                    lastFarTurnCommand = turnCommand;

                    if (liveBallRadius < 18.0) {
                        btask.step = farStepFast;
                    } else if (liveBallRadius < 25.0) {
                        btask.step = farStepMedium;
                    } else {
                        btask.step = farStepSlow;
                    }
                    btask.turn = turnCommand;
                    btask.count = farWalkCycleCount;
                } else if (liveTrackValid && liveLostFrames < chaseLostFrameLimit) {
                    // 短暂丢球时不追新噪声框，只以很低速度沿上一稳定趋势保持几帧。
                    btask.step = 0.015;
                    btask.turn = clampValue(
                        lastFarTurnCommand,
                        -1.2,
                        1.2
                    );
                    btask.count = 1;
                }
                break;

            case SoftPhase::NEAR_FORWARD:
                btask.step = nearForwardStep;
                btask.count = nearForwardCycleCount;
                break;

            default:
                break;
            }
        }

        // 只让“有限时长动作状态”按新相机帧倒计时；CHASE_FAR 是持续闭环，不使用倒计时。
        if (gameData.state == gameData.STATE_PLAY && robotUpright &&
            phase == phaseAtFrameStart &&
            motionFrameAdvance > 0 && phaseFrames > 0) {
            phaseFrames = std::max(0, phaseFrames - motionFrameAdvance);
        }

        // ---------- 有限运动状态的收口 ----------
        if (gameData.state == gameData.STATE_PLAY && robotUpright) {
            switch (phase) {
            case SoftPhase::SEARCH_BODY_TURN:
                if (phaseFrames == 0) {
                    phase = SoftPhase::SEARCH_BODY_SETTLE;
                    phaseFrames = turnSettleFrames;
                }
                break;

            case SoftPhase::SEARCH_BODY_SETTLE:
                if (phaseFrames == 0) {
                    enterSearchBall();
                }
                break;

            case SoftPhase::FACE_BALL_TURN:
                if (phaseFrames == 0) {
                    phase = SoftPhase::FACE_BALL_SETTLE;
                    phaseFrames = turnSettleFrames;
                }
                break;

            case SoftPhase::FACE_BALL_SETTLE:
                if (phaseFrames == 0) {
                    phase = SoftPhase::CHASE_REACQUIRE;
                    searchPoseMissFrames = 0;
                    resetStaticConfirmation();
                    headTargetYaw = 0.0;
                    headTargetPitch = 25.0;
                }
                break;

            case SoftPhase::NEAR_SETTLE:
                if (phaseFrames == 0) {
                    nearForwardPulses = 0;
                    enterNearReacquire();
                }
                break;

            case SoftPhase::NEAR_TURN:
                if (phaseFrames == 0) {
                    phase = SoftPhase::NEAR_TURN_SETTLE;
                    phaseFrames = turnSettleFrames;
                }
                break;

            case SoftPhase::NEAR_TURN_SETTLE:
                if (phaseFrames == 0) {
                    enterNearReacquire();
                }
                break;

            case SoftPhase::NEAR_FORWARD:
                if (phaseFrames == 0) {
                    ++nearForwardPulses;
                    phase = SoftPhase::NEAR_FORWARD_SETTLE;
                    phaseFrames = walkSettleFrames;
                }
                break;

            case SoftPhase::NEAR_FORWARD_SETTLE:
                if (phaseFrames == 0) {
                    enterNearReacquire();
                }
                break;

            case SoftPhase::RECOVER:
                if (robotUpright && newCameraFrame) {
                    recoverUprightFrames += std::max(1, motionFrameAdvance);
                    if (recoverUprightFrames >= 20) {
                        enterSearchBall();
                        recoverUprightFrames = 0;
                    }
                } else if (!robotUpright) {
                    recoverUprightFrames = 0;
                }
                break;

            default:
                break;
            }
        }

        // ---------- 调试画面 ----------
        if (!resultImage.empty()) {
            cv::line(
                resultImage,
                cv::Point(resultImage.cols / 2, 0),
                cv::Point(resultImage.cols / 2, resultImage.rows - 1),
                cv::Scalar(0, 255, 0),
                1
            );

            // 静止确认后的平均球圆用更粗线显示；运动期当前候选仍由 detectBall 画细圆。
            if ((phase == SoftPhase::V3_NEAR_READY_STOP ||
                 phase == SoftPhase::NEAR_REACQUIRE ||
                 phase == SoftPhase::CHASE_REACQUIRE ||
                 phase == SoftPhase::SEARCH_BALL) &&
                confirmedBallRadius > 0) {
                cv::circle(
                    resultImage,
                    confirmedBallCenter,
                    confirmedBallRadius,
                    cv::Scalar(255, 0, 255),
                    3
                );
            }

            const char *phaseName = "UNKNOWN";
            switch (phase) {
            case SoftPhase::WAIT_PLAY: phaseName = "WAIT_PLAY"; break;
            case SoftPhase::SEARCH_BALL: phaseName = "SEARCH_BALL"; break;
            case SoftPhase::SEARCH_BODY_TURN: phaseName = "SEARCH_BODY_TURN"; break;
            case SoftPhase::SEARCH_BODY_SETTLE: phaseName = "SEARCH_BODY_SETTLE"; break;
            case SoftPhase::FACE_BALL_TURN: phaseName = "FACE_BALL_TURN"; break;
            case SoftPhase::FACE_BALL_SETTLE: phaseName = "FACE_BALL_SETTLE"; break;
            case SoftPhase::CHASE_REACQUIRE: phaseName = "CHASE_REACQUIRE"; break;
            case SoftPhase::CHASE_FAR: phaseName = "CHASE_FAR"; break;
            case SoftPhase::NEAR_SETTLE: phaseName = "NEAR_SETTLE"; break;
            case SoftPhase::NEAR_REACQUIRE: phaseName = "NEAR_REACQUIRE"; break;
            case SoftPhase::NEAR_TURN: phaseName = "NEAR_TURN"; break;
            case SoftPhase::NEAR_TURN_SETTLE: phaseName = "NEAR_TURN_SETTLE"; break;
            case SoftPhase::NEAR_FORWARD: phaseName = "NEAR_FORWARD"; break;
            case SoftPhase::NEAR_FORWARD_SETTLE: phaseName = "NEAR_FORWARD_SETTLE"; break;
            case SoftPhase::V3_NEAR_READY_STOP: phaseName = "V3_NEAR_READY_STOP"; break;
            case SoftPhase::RECOVER: phaseName = "RECOVER"; break;
            }

            cv::putText(
                resultImage,
                phaseName,
                cv::Point(12, 28),
                cv::FONT_HERSHEY_SIMPLEX,
                0.65,
                cv::Scalar(255, 255, 0),
                2
            );

            const double shownX = liveTrackValid ? liveBallX : confirmedBallCenter.x;
            const double shownY = liveTrackValid ? liveBallY : confirmedBallCenter.y;
            const double shownR = liveTrackValid ? liveBallRadius : confirmedBallRadius;
            const double shownScore = currentObservationValid
                ? currentObservation.score
                : (liveTrackValid ? liveBallScore : confirmedBallScore);

            cv::putText(
                resultImage,
                cv::format(
                    "ball x=%.0f y=%.0f r=%.1f score=%.2f lost=%d",
                    shownX,
                    shownY,
                    shownR,
                    shownScore,
                    liveLostFrames
                ),
                cv::Point(12, 54),
                cv::FONT_HERSHEY_SIMPLEX,
                0.48,
                cv::Scalar(255, 255, 255),
                1
            );

            cv::putText(
                resultImage,
                cv::format(
                    "head=(%.1f,%.1f) imuYaw=%.2f loc(debug)=(%.2f,%.2f)",
                    static_cast<double>(headAngle.yaw),
                    static_cast<double>(headAngle.pitch),
                    static_cast<double>(imuData.yaw),
                    static_cast<double>(location.x),
                    static_cast<double>(location.z)
                ),
                cv::Point(12, 76),
                cv::FONT_HERSHEY_SIMPLEX,
                0.43,
                cv::Scalar(255, 255, 255),
                1
            );

            if (phase == SoftPhase::V3_NEAR_READY_STOP) {
                cv::putText(
                    resultImage,
                    "V3 PASS: autonomous chase reached safe near-ball zone",
                    cv::Point(12, 104),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.52,
                    cv::Scalar(0, 255, 0),
                    2
                );
            }

            resImgPublisher->Publish(resultImage);
        }

        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);

        loop_rate.sleep();
        // ----------------- 可以修改的部分 end--------------------
    }
    rclcpp::shutdown();
    return 0;
}

