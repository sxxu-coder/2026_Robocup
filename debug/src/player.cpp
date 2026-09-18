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
        // 固定预赛策略（红方）：
        // 1) 依据已知 C/O/A 几何完成两次 IMU 闭环转向和一次定长长走；
        // 2) 最终航向永久锁定为 O->A；
        // 3) 视觉只负责让球心对准右脚目标线，横移与前进严格串行；
        // 4) 横向对齐后只固定前进 0.05m；
        // 5) 停稳后不再做纵向视觉补偿或末端视觉复核，直接执行 right_kick。

        enum class ApproachPhase {
            WAIT_START,
            INITIAL_TURN,
            INITIAL_TURN_SETTLE,
            LONG_WALK,
            LONG_WALK_SETTLE,
            FINAL_TURN,
            FINAL_TURN_SETTLE,
            BALL_REACQUIRE,
            LATERAL_MOVE,
            LATERAL_SETTLE,
            LATERAL_YAW_RECOVER,
            LATERAL_YAW_SETTLE,
            FIXED_FORWARD_005,
            FIXED_FORWARD_SETTLE,
            KICK_REQUEST,
            KICK_WAIT,
            KICK_STOP
        };

        struct SearchPose {
            double yaw;
            double pitch;
        };

        static const std::vector<SearchPose> lateralSearchPoses = {
            {0.0, 45.0},
            {0.0, 60.0},
            {0.0, 35.0},
            {10.0, 45.0},
            {-10.0, 45.0}
        };

        // ---------- 固定场地几何 ----------
        // O=(0,0)，A=(-4.5,+0.91)。B 是右脚中心在 O->A 反向延长线上、距 O 0.25m 的点。
        // 右脚中心相对身体中心横向偏置按 0.07m 处理，反算身体中心的长走目标后：
        // 首次左转约 30.288019°，长走约 3.4522m；按 260帧≈3.23m 得 278帧初值。
        // O->A 相对初始朝向的绝对总左转角仍为约 101.432307°。
        static constexpr double nominalInitialLeftTurn = 30.288019;
        static constexpr double nominalAttackTurnFromStart = 101.432307;
        static constexpr int longWalkCommandFrames = 278;

        // ---------- 步态 / IMU 参数 ----------
        static constexpr int motionCommandFrames = 15;
        static constexpr int turnSettleFrames = 18;
        static constexpr int walkSettleFrames = 22;
        static constexpr int shortMoveSettleFrames = 24;
        static constexpr int turnCycleCount = 2;
        static constexpr int finalTurnInitialCycleCount = 3;
        static constexpr double turnAcceptDegrees = 1.5;
        static constexpr double yawRecoverAcceptDegrees = 1.0;
        static constexpr double maxYawResidualDegrees = 2.5;
        static constexpr int maxInitialTurnCorrections = 3;
        static constexpr int maxFinalTurnCorrections = 3;
        static constexpr int maxYawRecoverCorrections = 3;
        static constexpr double maxTurnCorrectionPerCycle = 8.0;
        static constexpr double longWalkStep = 0.05;
        static constexpr int longWalkCycleCount = 2;
        static constexpr double walkHeadingFilterAlpha = 0.15;
        static constexpr double walkHeadingDeadZone = 1.5;
        static constexpr double walkHeadingGain = 0.20;
        static constexpr double maxWalkTurnPerCycle = 2.5;

        // ---------- 近球视觉 / 右脚对齐参数 ----------
        static constexpr double rightFootLateralOffsetMetres = 0.07;
        static constexpr int ballConfirmFrames = 4;
        static constexpr int searchMissFramesPerPose = 10;
        static constexpr int maxLateralPulses = 5;
        static constexpr int lateralCycleCount = 2;
        static constexpr double lateralMaxPerCycle = 0.025;
        static constexpr double lateralMinPerCycle = 0.006;
        static constexpr double footWindowToleranceBallRadii = 0.25;
        static constexpr int minimumFootWindowTolerancePixels = 8;

        // ---------- 纵向策略 ----------
        // 横向对齐后只向前 0.05m（单周期），停稳后直接右脚踢球。
        static constexpr double fixedForwardStep = 0.05;
        static constexpr int fixedForwardCycleCount = 1;
        static constexpr int kickRequestFrames = 5;
        static constexpr int kickWaitFrames = 38;

        static ApproachPhase phase = ApproachPhase::WAIT_START;
        static int phaseFrames = 0;
        static bool initYawReady = false;
        static double initialYaw = 0.0;
        static bool imuTurnDirectionKnown = false;
        static double imuYawDirectionForPositiveTurn = 1.0;
        static double routeTargetYaw = 0.0;
        static double shotTargetYaw = 0.0;
        static double turnPulseStartYaw = 0.0;
        static double activeTurnCommand = 0.0;
        static int initialTurnCorrections = 0;
        static int finalTurnCorrections = 0;
        static int yawRecoverCorrections = 0;
        static bool finalTurnUsesThreeCycles = true;
        static double filteredWalkHeadingError = 0.0;
        static double walkTurnCommand = 0.0;

        static double headTargetYaw = 0.0;
        static double headTargetPitch = 20.0;
        static std::size_t searchPoseIndex = 0;
        static int searchMissFrames = 0;
        static int stableBallFrames = 0;
        static cv::Point lastBallCenter;
        static int lastBallRadius = 0;
        static double stableBallXSum = 0.0;
        static double stableBallYSum = 0.0;
        static double stableBallRadiusSum = 0.0;
        static cv::Point trackedBallCenter;
        static int trackedBallRadius = 0;
        static int preferredFootTargetX = -1;
        static int preferredFootTolerancePixels = 0;
        static int lateralPulses = 0;
        static double lateralCommand = 0.0;
        static bool kickIssued = false;

        static const auto normalizeAngle = [](double angle) {
            while (angle > 180.0) angle -= 360.0;
            while (angle <= -180.0) angle += 360.0;
            return angle;
        };

        static const auto clampValue = [](double value, double low, double high) {
            return std::max(low, std::min(high, value));
        };

        const auto physicalYawError = [&](double targetYaw) {
            return normalizeAngle(
                targetYaw - static_cast<double>(imuData.yaw)
            ) * imuYawDirectionForPositiveTurn;
        };

        // 仿真倍率可能变化，因此所有“帧数时序”只按真正收到的新相机帧推进。
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
        }
        const int motionFrameAdvance = static_cast<int>(
            std::min<unsigned long long>(newCameraFrameCount, 100ULL)
        );

        // 每轮先清空 BodyTask，只有当前状态明确需要运动时才重新赋值。
        btask.type = btask.TASK_WALK;
        btask.step = 0.0;
        btask.lateral = 0.0;
        btask.turn = 0.0;
        btask.count = 0;
        htask.yaw = headTargetYaw;
        htask.pitch = headTargetPitch;
        const bool robotUpright = imuData.fall == imuData.FALL_NONE;

        const auto resetBallConfirmation = [&]() {
            stableBallFrames = 0;
            lastBallCenter = cv::Point();
            lastBallRadius = 0;
            stableBallXSum = 0.0;
            stableBallYSum = 0.0;
            stableBallRadiusSum = 0.0;
            trackedBallCenter = cv::Point();
            trackedBallRadius = 0;
            preferredFootTargetX = -1;
            preferredFootTolerancePixels = 0;
            searchMissFrames = 0;
        };

        const auto setSearchPose = [&](const std::vector<SearchPose> &poses) {
            if (searchPoseIndex < poses.size()) {
                headTargetYaw = poses[searchPoseIndex].yaw;
                headTargetPitch = poses[searchPoseIndex].pitch;
                htask.yaw = headTargetYaw;
                htask.pitch = headTargetPitch;
            }
        };

        const auto beginLateralReacquire = [&]() {
            phase = ApproachPhase::BALL_REACQUIRE;
            searchPoseIndex = 0;
            resetBallConfirmation();
            setSearchPose(lateralSearchPoses);
        };

        const auto requestKick = [&]() {
            kickIssued = true;
            phase = ApproachPhase::KICK_REQUEST;
            phaseFrames = kickRequestFrames;
            headTargetYaw = 0.0;
            headTargetPitch = 45.0;
        };

        // 仅用于近球静止阶段。Hough 圆给出几何，再用黑白球内部与绿色外环抑制场地线误检。
        const auto detectNearBall = [&](cv::Point &centerOut, int &radiusOut,
                                        cv::Mat &debugImage) {
            if (image.empty()) return false;

            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
            cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2.0, 2.0);

            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(
                gray,
                circles,
                cv::HOUGH_GRADIENT,
                1.2,
                24.0,
                100.0,
                14.0,
                20,
                std::min(130, std::min(image.cols, image.rows) / 3)
            );

            cv::Mat hsv;
            cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV);
            cv::Mat whiteMask;
            cv::Mat darkMask;
            cv::Mat greenMask;
            cv::inRange(hsv, cv::Scalar(0, 0, 140), cv::Scalar(180, 95, 255), whiteMask);
            cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 120, 125), darkMask);
            cv::inRange(hsv, cv::Scalar(25, 50, 25), cv::Scalar(95, 255, 255), greenMask);

            bool found = false;
            double bestScore = -1.0;
            cv::Point bestCenter;
            int bestRadius = 0;

            for (const auto &c : circles) {
                const cv::Point center(cvRound(c[0]), cvRound(c[1]));
                const int radius = cvRound(c[2]);
                if (radius < 20) continue;
                const int outerRadius = cvRound(radius * 1.40);
                if (center.x - outerRadius < 2 || center.y - outerRadius < 2 ||
                    center.x + outerRadius >= image.cols - 2 ||
                    center.y + outerRadius >= image.rows - 2) {
                    continue;
                }

                const cv::Rect roiRect(
                    center.x - outerRadius,
                    center.y - outerRadius,
                    outerRadius * 2 + 1,
                    outerRadius * 2 + 1
                );
                const cv::Point localCenter = center - roiRect.tl();
                cv::Mat inner = cv::Mat::zeros(roiRect.size(), CV_8UC1);
                cv::Mat ringOuter = cv::Mat::zeros(roiRect.size(), CV_8UC1);
                cv::Mat ringInner = cv::Mat::zeros(roiRect.size(), CV_8UC1);
                cv::circle(inner, localCenter, std::max(1, cvRound(radius * 0.88)),
                           cv::Scalar(255), -1);
                cv::circle(ringOuter, localCenter, outerRadius, cv::Scalar(255), -1);
                cv::circle(ringInner, localCenter, std::max(1, cvRound(radius * 1.05)),
                           cv::Scalar(255), -1);
                cv::Mat ring;
                cv::subtract(ringOuter, ringInner, ring);

                const int innerPixels = cv::countNonZero(inner);
                const int ringPixels = cv::countNonZero(ring);
                if (innerPixels <= 0 || ringPixels <= 0) continue;

                cv::Mat sampled;
                cv::bitwise_and(whiteMask(roiRect), inner, sampled);
                const double whiteRatio = static_cast<double>(cv::countNonZero(sampled)) /
                    static_cast<double>(innerPixels);
                cv::bitwise_and(darkMask(roiRect), inner, sampled);
                const double darkRatio = static_cast<double>(cv::countNonZero(sampled)) /
                    static_cast<double>(innerPixels);
                cv::bitwise_and(greenMask(roiRect), ring, sampled);
                const double greenRingRatio = static_cast<double>(cv::countNonZero(sampled)) /
                    static_cast<double>(ringPixels);

                if (whiteRatio < 0.22 || darkRatio < 0.025 || greenRingRatio < 0.18) {
                    continue;
                }

                const double score =
                    2.5 * whiteRatio +
                    2.0 * std::min(darkRatio / 0.20, 1.0) +
                    2.5 * greenRingRatio +
                    std::min(static_cast<double>(radius) / 90.0, 1.0);
                if (!found || score > bestScore) {
                    found = true;
                    bestScore = score;
                    bestCenter = center;
                    bestRadius = radius;
                }
            }

            if (found) {
                centerOut = bestCenter;
                radiusOut = bestRadius;
                cv::circle(debugImage, bestCenter, bestRadius, cv::Scalar(0, 255, 255), 2);
                cv::circle(debugImage, bestCenter, 3, cv::Scalar(255, 0, 0), -1);
            }
            return found;
        };

        const auto acceptBallObservation = [&](const cv::Point &center, int radius) {
            const bool continuous = stableBallFrames > 0 && lastBallRadius > 0 &&
                cv::norm(center - lastBallCenter) <= std::max(12.0, 0.25 * radius) &&
                std::abs(radius - lastBallRadius) <= std::max(6.0, 0.20 * radius);

            if (!continuous) {
                stableBallFrames = 1;
                stableBallXSum = center.x;
                stableBallYSum = center.y;
                stableBallRadiusSum = radius;
            } else {
                ++stableBallFrames;
                stableBallXSum += center.x;
                stableBallYSum += center.y;
                stableBallRadiusSum += radius;
            }
            lastBallCenter = center;
            lastBallRadius = radius;

            if (stableBallFrames >= ballConfirmFrames) {
                trackedBallCenter = cv::Point(
                    cvRound(stableBallXSum / stableBallFrames),
                    cvRound(stableBallYSum / stableBallFrames)
                );
                trackedBallRadius = std::max(1, cvRound(
                    stableBallRadiusSum / stableBallFrames
                ));
                preferredFootTargetX = image.cols / 2 + trackedBallRadius;
                preferredFootTolerancePixels = std::max(
                    minimumFootWindowTolerancePixels,
                    cvRound(footWindowToleranceBallRadii * trackedBallRadius)
                );
                return true;
            }
            return false;
        };

        const auto planLateralPulse = [&]() {
            if (trackedBallRadius <= 0 || preferredFootTargetX < 0) return false;
            const int pixelError = trackedBallCenter.x - preferredFootTargetX;
            if (std::abs(pixelError) <= preferredFootTolerancePixels) {
                lateralCommand = 0.0;
                return true;
            }
            if (lateralPulses >= maxLateralPulses) {
                return false;
            }

            // (像素横差 / 球像素半径) ≈ (物理横差 / 0.07m)。
            // 两个横移周期共同消除估计横差，因此每周期取一半并保守限幅。
            const double estimatedPerCycle =
                -0.5 * rightFootLateralOffsetMetres *
                static_cast<double>(pixelError) /
                static_cast<double>(trackedBallRadius);
            const double magnitude = clampValue(
                std::abs(estimatedPerCycle),
                lateralMinPerCycle,
                lateralMaxPerCycle
            );
            lateralCommand = estimatedPerCycle >= 0.0 ? magnitude : -magnitude;
            ++lateralPulses;
            phase = ApproachPhase::LATERAL_MOVE;
            phaseFrames = motionCommandFrames;
            return true;
        };

        const auto startYawRecovery = [&](ApproachPhase recoverPhase,
                                          double targetYaw,
                                          int &correctionCounter) {
            const double error = physicalYawError(targetYaw);
            if (std::abs(error) <= yawRecoverAcceptDegrees) {
                return false;
            }
            if (correctionCounter >= maxYawRecoverCorrections) {
                return false;
            }
            activeTurnCommand = clampValue(
                error / static_cast<double>(turnCycleCount),
                -maxTurnCorrectionPerCycle,
                maxTurnCorrectionPerCycle
            );
            ++correctionCounter;
            phase = recoverPhase;
            phaseFrames = motionCommandFrames;
            return true;
        };

        // INIT 是唯一会彻底重置整条策略的状态。
        if (gameData.state == gameData.STATE_INIT) {
            initialYaw = static_cast<double>(imuData.yaw);
            initYaw = static_cast<float>(initialYaw);
            initYawReady = true;

            phase = ApproachPhase::WAIT_START;
            phaseFrames = 0;
            imuTurnDirectionKnown = false;
            imuYawDirectionForPositiveTurn = 1.0;
            routeTargetYaw = initialYaw;
            shotTargetYaw = initialYaw;
            activeTurnCommand = 0.0;
            initialTurnCorrections = 0;
            finalTurnCorrections = 0;
            yawRecoverCorrections = 0;
            finalTurnUsesThreeCycles = true;
            filteredWalkHeadingError = 0.0;
            walkTurnCommand = 0.0;
            lateralPulses = 0;
            lateralCommand = 0.0;
            kickIssued = false;
            searchPoseIndex = 0;
            headTargetYaw = 0.0;
            headTargetPitch = 20.0;
            resetBallConfirmation();
        }

        // PLAY 首次进入：直接启动固定几何的第一次左转。
        if (gameData.state == gameData.STATE_PLAY &&
            phase == ApproachPhase::WAIT_START && robotUpright) {
            if (!initYawReady) {
                initialYaw = static_cast<double>(imuData.yaw);
                initYaw = static_cast<float>(initialYaw);
                initYawReady = true;
            }
            if (myColor != COLOR_RED) {
                RCLCPP_ERROR(playerNode->get_logger(),
                    "This fixed-geometry strategy is calibrated for the red side only.");
                phase = ApproachPhase::KICK_STOP;
            } else {
                turnPulseStartYaw = static_cast<double>(imuData.yaw);
                activeTurnCommand = nominalInitialLeftTurn /
                    static_cast<double>(turnCycleCount);
                phase = ApproachPhase::INITIAL_TURN;
                phaseFrames = motionCommandFrames;
                headTargetYaw = 0.0;
                headTargetPitch = 20.0;
            }
        }

        // 非 PLAY 时保持零运动；如果踢球后 gamectrl 已暂停/结束，则正常收口。
        if (gameData.state != gameData.STATE_PLAY && kickIssued &&
            (phase == ApproachPhase::KICK_REQUEST ||
             phase == ApproachPhase::KICK_WAIT)) {
            phase = ApproachPhase::KICK_STOP;
        }

        // 长走时只闭环保持第一次转向后的固定航向。
        if (gameData.state == gameData.STATE_PLAY && robotUpright &&
            phase == ApproachPhase::LONG_WALK && imuTurnDirectionKnown) {
            const double rawError = physicalYawError(routeTargetYaw);
            if (newCameraFrame) {
                const double alpha = 1.0 - std::pow(
                    1.0 - walkHeadingFilterAlpha,
                    static_cast<double>(std::max(1, motionFrameAdvance))
                );
                filteredWalkHeadingError =
                    (1.0 - alpha) * filteredWalkHeadingError + alpha * rawError;
            }
            walkTurnCommand = 0.0;
            if (std::abs(filteredWalkHeadingError) > walkHeadingDeadZone) {
                walkTurnCommand = clampValue(
                    walkHeadingGain * filteredWalkHeadingError,
                    -maxWalkTurnPerCycle,
                    maxWalkTurnPerCycle
                );
            }
        }

        const ApproachPhase phaseAtCommandSelection = phase;

        // ---------- 当前状态对应的 BodyTask ----------
        if (gameData.state == gameData.STATE_PLAY && robotUpright) {
            switch (phase) {
            case ApproachPhase::INITIAL_TURN:
            case ApproachPhase::LATERAL_YAW_RECOVER:
                btask.turn = activeTurnCommand;
                btask.count = turnCycleCount;
                break;
            case ApproachPhase::FINAL_TURN:
                btask.turn = activeTurnCommand;
                btask.count = finalTurnUsesThreeCycles
                    ? finalTurnInitialCycleCount : turnCycleCount;
                break;
            case ApproachPhase::LONG_WALK:
                btask.step = longWalkStep;
                btask.turn = walkTurnCommand;
                btask.count = longWalkCycleCount;
                break;
            case ApproachPhase::LATERAL_MOVE:
                btask.lateral = lateralCommand;
                btask.count = lateralCycleCount;
                break;
            case ApproachPhase::FIXED_FORWARD_005:
                btask.step = fixedForwardStep;
                btask.count = fixedForwardCycleCount;
                break;
            case ApproachPhase::KICK_REQUEST:
                btask.type = btask.TASK_ACT;
                btask.actname = "right_kick";
                btask.count = 1;
                break;
            default:
                break;
            }
        }

        // ---------- 静止视觉 ----------
        cv::Mat resultImage = image.empty() ? cv::Mat() : image.clone();
        const bool visualPhase =
            phase == ApproachPhase::BALL_REACQUIRE;
        if (gameData.state == gameData.STATE_PLAY && robotUpright &&
            newCameraFrame && visualPhase && !image.empty()) {
            const bool headAtTarget =
                std::abs(static_cast<double>(headAngle.yaw) - headTargetYaw) <= 3.0 &&
                std::abs(static_cast<double>(headAngle.pitch) - headTargetPitch) <= 3.0;

            if (headAtTarget) {
                cv::Point candidateCenter;
                int candidateRadius = 0;
                const bool found = detectNearBall(
                    candidateCenter,
                    candidateRadius,
                    resultImage
                );

                if (found) {
                    searchMissFrames = 0;
                    const bool confirmed = acceptBallObservation(
                        candidateCenter,
                        candidateRadius
                    );
                    if (confirmed) {
                        const int pixelError = trackedBallCenter.x - preferredFootTargetX;
                        if (std::abs(pixelError) <= preferredFootTolerancePixels) {
                            lateralCommand = 0.0;
                            phase = ApproachPhase::FIXED_FORWARD_005;
                            phaseFrames = motionCommandFrames;
                            headTargetYaw = 0.0;
                            headTargetPitch = 45.0;
                            RCLCPP_INFO(playerNode->get_logger(),
                                "Right-foot line aligned. Start fixed 0.05m forward motion.");
                        } else if (!planLateralPulse()) {
                            RCLCPP_ERROR(playerNode->get_logger(),
                                "Lateral alignment failed after %d pulses.", lateralPulses);
                            phase = ApproachPhase::KICK_STOP;
                        }
                    }
                } else {
                    stableBallFrames = 0;
                    stableBallXSum = 0.0;
                    stableBallYSum = 0.0;
                    stableBallRadiusSum = 0.0;
                    ++searchMissFrames;
                    if (searchMissFrames >= searchMissFramesPerPose) {
                        searchMissFrames = 0;
                        ++searchPoseIndex;
                        const auto &poses = lateralSearchPoses;
                        if (searchPoseIndex >= poses.size()) {
                            RCLCPP_ERROR(playerNode->get_logger(),
                                "Ball reacquire failed during lateral alignment.");
                            phase = ApproachPhase::KICK_STOP;
                        } else {
                            stableBallFrames = 0;
                            setSearchPose(poses);
                        }
                    }
                }
            }
        }

        // ---------- 基于新相机帧推进运动状态 ----------
        if (gameData.state == gameData.STATE_PLAY && robotUpright &&
            phase == phaseAtCommandSelection &&
            motionFrameAdvance > 0 && phaseFrames > 0) {
            phaseFrames = std::max(0, phaseFrames - motionFrameAdvance);
        }

        if (gameData.state == gameData.STATE_PLAY && robotUpright) {
            switch (phase) {
            case ApproachPhase::INITIAL_TURN:
                if (phaseFrames == 0) {
                    phase = ApproachPhase::INITIAL_TURN_SETTLE;
                    phaseFrames = turnSettleFrames;
                }
                break;

            case ApproachPhase::INITIAL_TURN_SETTLE:
                if (phaseFrames == 0) {
                    if (!imuTurnDirectionKnown) {
                        const double yawDelta = normalizeAngle(
                            static_cast<double>(imuData.yaw) - turnPulseStartYaw
                        );
                        if (std::abs(yawDelta) < 2.0) {
                            RCLCPP_ERROR(playerNode->get_logger(),
                                "Initial turn did not produce measurable IMU yaw change.");
                            phase = ApproachPhase::KICK_STOP;
                            break;
                        }
                        imuYawDirectionForPositiveTurn = yawDelta >= 0.0 ? 1.0 : -1.0;
                        imuTurnDirectionKnown = true;
                        routeTargetYaw = normalizeAngle(
                            initialYaw +
                            imuYawDirectionForPositiveTurn * nominalInitialLeftTurn
                        );
                    }

                    const double error = physicalYawError(routeTargetYaw);
                    if (std::abs(error) > turnAcceptDegrees &&
                        initialTurnCorrections < maxInitialTurnCorrections) {
                        activeTurnCommand = clampValue(
                            error / static_cast<double>(turnCycleCount),
                            -maxTurnCorrectionPerCycle,
                            maxTurnCorrectionPerCycle
                        );
                        ++initialTurnCorrections;
                        phase = ApproachPhase::INITIAL_TURN;
                        phaseFrames = motionCommandFrames;
                    } else if (std::abs(error) > maxYawResidualDegrees) {
                        RCLCPP_ERROR(playerNode->get_logger(),
                            "Initial geometry turn residual too large: %.2f deg", error);
                        phase = ApproachPhase::KICK_STOP;
                    } else {
                        filteredWalkHeadingError = error;
                        phase = ApproachPhase::LONG_WALK;
                        phaseFrames = longWalkCommandFrames;
                        RCLCPP_INFO(playerNode->get_logger(),
                            "Initial turn complete. Start %d-frame long walk.",
                            longWalkCommandFrames);
                    }
                }
                break;

            case ApproachPhase::LONG_WALK:
                if (phaseFrames == 0) {
                    phase = ApproachPhase::LONG_WALK_SETTLE;
                    phaseFrames = walkSettleFrames;
                }
                break;

            case ApproachPhase::LONG_WALK_SETTLE:
                if (phaseFrames == 0) {
                    shotTargetYaw = normalizeAngle(
                        initialYaw +
                        imuYawDirectionForPositiveTurn * nominalAttackTurnFromStart
                    );
                    const double error = physicalYawError(shotTargetYaw);
                    activeTurnCommand = clampValue(
                        error / static_cast<double>(finalTurnInitialCycleCount),
                        -25.0,
                        25.0
                    );
                    finalTurnUsesThreeCycles = true;
                    finalTurnCorrections = 0;
                    phase = ApproachPhase::FINAL_TURN;
                    phaseFrames = motionCommandFrames;
                }
                break;

            case ApproachPhase::FINAL_TURN:
                if (phaseFrames == 0) {
                    phase = ApproachPhase::FINAL_TURN_SETTLE;
                    phaseFrames = turnSettleFrames;
                }
                break;

            case ApproachPhase::FINAL_TURN_SETTLE:
                if (phaseFrames == 0) {
                    const double error = physicalYawError(shotTargetYaw);
                    if (std::abs(error) > turnAcceptDegrees &&
                        finalTurnCorrections < maxFinalTurnCorrections) {
                        activeTurnCommand = clampValue(
                            error / static_cast<double>(turnCycleCount),
                            -maxTurnCorrectionPerCycle,
                            maxTurnCorrectionPerCycle
                        );
                        ++finalTurnCorrections;
                        finalTurnUsesThreeCycles = false;
                        phase = ApproachPhase::FINAL_TURN;
                        phaseFrames = motionCommandFrames;
                    } else if (std::abs(error) > maxYawResidualDegrees) {
                        RCLCPP_ERROR(playerNode->get_logger(),
                            "Final AO turn residual too large: %.2f deg", error);
                        phase = ApproachPhase::KICK_STOP;
                    } else {
                        // 注意：后续永远锁定理论 AO yaw，不把当前实际 yaw 覆盖成新目标。
                        lateralPulses = 0;
                        beginLateralReacquire();
                    }
                }
                break;

            case ApproachPhase::LATERAL_MOVE:
                if (phaseFrames == 0) {
                    phase = ApproachPhase::LATERAL_SETTLE;
                    phaseFrames = shortMoveSettleFrames;
                }
                break;

            case ApproachPhase::LATERAL_SETTLE:
                if (phaseFrames == 0) {
                    yawRecoverCorrections = 0;
                    if (!startYawRecovery(
                            ApproachPhase::LATERAL_YAW_RECOVER,
                            shotTargetYaw,
                            yawRecoverCorrections)) {
                        if (std::abs(physicalYawError(shotTargetYaw)) > maxYawResidualDegrees) {
                            phase = ApproachPhase::KICK_STOP;
                        } else {
                            beginLateralReacquire();
                        }
                    }
                }
                break;

            case ApproachPhase::LATERAL_YAW_RECOVER:
                if (phaseFrames == 0) {
                    phase = ApproachPhase::LATERAL_YAW_SETTLE;
                    phaseFrames = turnSettleFrames;
                }
                break;

            case ApproachPhase::LATERAL_YAW_SETTLE:
                if (phaseFrames == 0) {
                    if (!startYawRecovery(
                            ApproachPhase::LATERAL_YAW_RECOVER,
                            shotTargetYaw,
                            yawRecoverCorrections)) {
                        if (std::abs(physicalYawError(shotTargetYaw)) > maxYawResidualDegrees) {
                            phase = ApproachPhase::KICK_STOP;
                        } else {
                            beginLateralReacquire();
                        }
                    }
                }
                break;

            case ApproachPhase::FIXED_FORWARD_005:
                if (phaseFrames == 0) {
                    phase = ApproachPhase::FIXED_FORWARD_SETTLE;
                    phaseFrames = shortMoveSettleFrames;
                }
                break;

            case ApproachPhase::FIXED_FORWARD_SETTLE:
                if (phaseFrames == 0) {
                    // 横向对齐后只前进 0.05m；停稳后不再估距、不再补偿、
                    // 不在球前做额外转向，直接执行右脚踢球。
                    requestKick();
                }
                break;

            case ApproachPhase::KICK_REQUEST:
                if (phaseFrames == 0) {
                    phase = ApproachPhase::KICK_WAIT;
                    phaseFrames = kickWaitFrames;
                }
                break;

            case ApproachPhase::KICK_WAIT:
                if (phaseFrames == 0) {
                            phase = ApproachPhase::KICK_STOP;
                }
                break;

            default:
                break;
            }
        }

        // 跌倒时立即停止继续推进状态机，保持默认零步态命令。
        if (!robotUpright && gameData.state == gameData.STATE_PLAY) {
            if (phase != ApproachPhase::KICK_WAIT &&
                phase != ApproachPhase::KICK_STOP) {
                RCLCPP_ERROR(playerNode->get_logger(), "Robot fall detected; stop strategy.");
                phase = ApproachPhase::KICK_STOP;
            }
        }

        // ---------- 调试画面：身体中心线、右脚目标线、球圆和当前阶段 ----------
        if (!resultImage.empty()) {
            cv::line(
                resultImage,
                cv::Point(resultImage.cols / 2, 0),
                cv::Point(resultImage.cols / 2, resultImage.rows - 1),
                cv::Scalar(0, 255, 0),
                1
            );
            // trackedBall* 只在静止重捕获时更新；运动阶段不绘制旧框，
            // 避免调试图上出现“识别框停在旧位置”的假象。
            if (phase == ApproachPhase::BALL_REACQUIRE && trackedBallRadius > 0) {
                const int footX = resultImage.cols / 2 + trackedBallRadius;
                cv::line(
                    resultImage,
                    cv::Point(footX, 0),
                    cv::Point(footX, resultImage.rows - 1),
                    cv::Scalar(255, 0, 255),
                    2
                );
                cv::circle(
                    resultImage,
                    trackedBallCenter,
                    trackedBallRadius,
                    cv::Scalar(0, 255, 255),
                    2
                );
            }

            const char *phaseName = "UNKNOWN";
            switch (phase) {
            case ApproachPhase::WAIT_START: phaseName = "WAIT_START"; break;
            case ApproachPhase::INITIAL_TURN: phaseName = "INITIAL_TURN"; break;
            case ApproachPhase::INITIAL_TURN_SETTLE: phaseName = "INITIAL_TURN_SETTLE"; break;
            case ApproachPhase::LONG_WALK: phaseName = "LONG_WALK"; break;
            case ApproachPhase::LONG_WALK_SETTLE: phaseName = "LONG_WALK_SETTLE"; break;
            case ApproachPhase::FINAL_TURN: phaseName = "FINAL_TURN"; break;
            case ApproachPhase::FINAL_TURN_SETTLE: phaseName = "FINAL_TURN_SETTLE"; break;
            case ApproachPhase::BALL_REACQUIRE: phaseName = "BALL_REACQUIRE"; break;
            case ApproachPhase::LATERAL_MOVE: phaseName = "LATERAL_MOVE"; break;
            case ApproachPhase::LATERAL_SETTLE: phaseName = "LATERAL_SETTLE"; break;
            case ApproachPhase::LATERAL_YAW_RECOVER: phaseName = "LATERAL_YAW_RECOVER"; break;
            case ApproachPhase::LATERAL_YAW_SETTLE: phaseName = "LATERAL_YAW_SETTLE"; break;
            case ApproachPhase::FIXED_FORWARD_005: phaseName = "FIXED_FORWARD_005"; break;
            case ApproachPhase::FIXED_FORWARD_SETTLE: phaseName = "FIXED_FORWARD_SETTLE"; break;
            case ApproachPhase::KICK_REQUEST: phaseName = "KICK_REQUEST"; break;
            case ApproachPhase::KICK_WAIT: phaseName = "KICK_WAIT"; break;
            case ApproachPhase::KICK_STOP: phaseName = "KICK_STOP"; break;
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

