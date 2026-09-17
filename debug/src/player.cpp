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
        // 所有新增状态和接口都严格保留在赛方允许修改的区域内。
        // static 使状态跨主循环保存，不需要在 begin/end 之外增加全局变量。
        static bool initYawReady = false;
        static bool hasBallTrack = false;
        static cv::Point trackedBallCenter;
        static cv::Rect trackedBallRect;
        static int lostBallFrames = 0;
        static int settledLostBallFrames = 0;
        static int stableBallFrames = 0;
        static int liveBallFrames = 0;

        struct SearchPose {
            double yaw;
            double pitch;
        };
        // 已实测的近球重捕获从 25° 开始；其他姿态仅在首选姿态
        // 未找到球时尝试，不改变 T3 射门航向。
        static const std::vector<SearchPose> localReacquirePoses = {
            {0.0, 25.0},
            {0.0, 35.0},
            {10.0, 35.0},
            {-10.0, 35.0},
            {0.0, 45.0},
            {10.0, 45.0},
            {-10.0, 45.0}
        };
        static std::size_t localReacquirePoseIndex = 0;
        static int previousGameState = -1;
        static double headTargetYaw = 0.0;
        static double headTargetPitch = 20.0;

        enum class ApproachPhase {
            WAIT_INITIAL_TURN,
            OBSERVE,
            BODY_TURN,
            TURN_SETTLE,
            LONG_WALK,
            WALK_SETTLE,
            BALL_FACING_TURN,
            BALL_FACING_SETTLE,
            GOAL_VIEW_TEST,
            SHOT_TARGET_TURN,
            SHOT_TARGET_SETTLE,
            SHOT_TARGET_STOP,
            BALL_REACQUIRE,
            BALL_REACQUIRE_STOP,
            COARSE_LATERAL_MOVE,
            COARSE_LATERAL_SETTLE,
            COARSE_YAW_RECOVER,
            COARSE_YAW_SETTLE,
            COARSE_BALL_REACQUIRE,
            COARSE_LATERAL_STOP,
            FORWARD_CALIBRATION_MOVE,
            FORWARD_CALIBRATION_SETTLE,
            FORWARD_YAW_RECOVER,
            FORWARD_YAW_SETTLE,
            FORWARD_BALL_REACQUIRE,
            FORWARD_CALIBRATION_STOP,
            FINE_MOVE,
            FINE_SETTLE,
            FINE_YAW_RECOVER,
            FINE_YAW_SETTLE,
            FINE_BALL_REACQUIRE,
            KICK_REQUEST,
            KICK_WAIT,
            // 一次性踢球动作后的硬终点，或安全检查失败时的停止点。
            KICK_STOP
        };
        static ApproachPhase approachPhase = ApproachPhase::WAIT_INITIAL_TURN;
        static int approachPhaseFrames = 0;
        static bool reacquireAfterTurn = false;
        static int reacquireMatchFrames = 0;
        static int reacquireMissFrames = 0;
        static int goalViewStableFrames = 0;
        static int goalViewLocationSamples = 0;
        static double goalViewLocationXSum = 0.0;
        static double goalViewLocationZSum = 0.0;
        static int playStartRemainTime = -1;
        static int longWalkStartRemainTime = -1;
        static int routeCompleteRemainTime = -1;
        static bool hasGoalTrack = false;
        static cv::Rect trackedGoalRect;
        static int goalStableFrames = 0;
        static int goalMissFrames = 0;
        static bool hasKeeperTrack = false;
        static cv::Rect trackedKeeperRect;
        static int keeperStableFrames = 0;
        static int keeperMissFrames = 0;
        static bool t1KeeperAnchorVisible = false;
        static cv::Rect t1KeeperAnchorRect;
        static int t1VerticalLineCandidates = 0;
        static int t1GoalDetectionMethod = 0;
        static bool keeperMotionReady = false;
        static double filteredKeeperU = 0.5;
        static double leftShotClearance = 0.0;
        static double rightShotClearance = 0.0;
        static int keeperMotionSamples = 0;
        static int pendingShotSide = 0;
        static int shotSideStableFrames = 0;
        static int lockedShotSide = 0;
        static bool hasFrozenShotGoal = false;
        static cv::Rect frozenShotGoalRect;
        static int shotDecisionWaitFrames = 0;
        static double shotTargetVisualAngleDegrees = 0.0;
        static double shotTargetAimAngleDegrees = 0.0;
        static double shotTargetImuYaw = 0.0;
        static double shotTargetPhysicalError = 0.0;
        static int shotTargetCorrectionPulses = 0;
        static bool shotTargetAlignmentSafetyStop = false;
        static bool ballReacquireSucceeded = false;
        static bool ballReacquireSafetyStop = false;
        static double ballReacquireHeadYaw = 0.0;
        static double ballReacquireHeadPitch = 0.0;
        static int nearBallGeometrySamples = 0;
        static double nearBallCenterXSum = 0.0;
        static double nearBallCenterYSum = 0.0;
        static double nearBallRadiusSum = 0.0;
        static int nearBallConfirmedRadius = 0;
        static double coarseLateralCommand = 0.0;
        static int coarseInitialBallX = -1;
        static int coarseBeforeBallX = -1;
        static int coarseBeforeTargetX = -1;
        static int coarseAfterBallX = -1;
        static int coarsePixelShift = 0;
        static int preferredFootTargetX = -1;
        static int preferredFootTolerancePixels = 0;
        static int coarseLateralPulses = 0;
        static double coarseHeadingError = 0.0;
        static int coarseYawCorrectionPulses = 0;
        static bool coarseDirectionMatched = false;
        static bool coarseAlignmentReached = false;
        static bool coarseSafetyStop = false;
        static bool forwardCalibrationStarted = false;
        static bool forwardReacquireSucceeded = false;
        static bool forwardSafetyStop = false;
        static int forwardBeforeBallX = -1;
        static int forwardBeforeBallY = -1;
        static int forwardBeforeRadius = 0;
        static int forwardAfterBallX = -1;
        static int forwardAfterBallY = -1;
        static int forwardAfterRadius = 0;
        static int forwardDeltaBallY = 0;
        static int forwardDeltaRadius = 0;
        static double forwardBeforeHeadingError = 0.0;
        static double forwardAfterHeadingError = 0.0;
        static int forwardYawCorrectionPulses = 0;
        static double fineStepCommand = 0.0;
        static double fineLateralCommand = 0.0;
        static int fineForwardPulses = 0;
        static int fineLateralPulses = 0;
        static int fineYawCorrectionPulses = 0;
        static bool fineSafetyStop = false;
        static const char *fineStopReason = "unknown";
        static int finalBallX = -1;
        static int finalBallY = -1;
        static int finalBallRadius = 0;
        static double finalHeadingError = 0.0;
        static bool kickIssued = false;
        static bool kickWaitCompleted = false;
        static bool kickFallObserved = false;
        static int kickStartRemainTime = -1;
        static int kickStopRemainTime = -1;

        // 红方固定起点 (2, 3) 到中心球正后方射门预备点
        // (0.5, 0) 的理论左转角。
        // 实际闭环只使用规则允许的 IMU yaw，不读取 Webots 世界坐标。
        static constexpr double nominalInitialLeftTurn = 26.565051;
        static double activeTurnCommand = 0.0;
        static double turnPulseStartYaw = 0.0;
        static double targetImuYaw = 0.0;
        static double imuYawDirectionForPositiveTurn = 1.0;
        static bool imuTurnDirectionKnown = false;
        static int turnPulseNumber = 0;
        static bool initialGeometryTurnComplete = false;
        static bool initialTurnSafetyStop = false;
        static double latestPhysicalTurnError = nominalInitialLeftTurn;
        static bool routeTravelStarted = false;
        static bool routeTravelComplete = false;
        static int longWalkProcessedFrames = 0;
        static double filteredWalkHeadingError = 0.0;
        static double walkTurnCommand = 0.0;
        static double finalWalkHeadingError = 0.0;
        static bool ballFacingTurnComplete = false;
        static double ballFacingTargetImuYaw = 0.0;
        static double ballFacingPhysicalError = 0.0;
        static int ballFacingCorrectionPulses = 0;
        static bool ballFacingAlignmentSafetyStop = false;

        // motion 首次收到行走任务会先生成原地起步周期，因此转向命令需
        // 保持足够多的新相机帧，确保动作缓存真正收到该命令。
        static constexpr int turnCommandFrames = 15;
        static constexpr int turnSettleFrames = 18;
        static constexpr int precisionTurnSettleFrames = 18;
        static constexpr int turnCycleCount = 2;
        // 以相机新帧而非墙钟计时，仿真窗口显示的实时倍率波动不会改变
        // 状态机的仿真时序。
        static constexpr double longWalkStep = 0.05;
        static constexpr int longWalkCycleCount = 2;
        // 2026-09-10 俯视图证明“320 帧约 3 m”的旧估算不成立：
        // 358 帧已越过中心球约 1 m；260 帧基线实测落在球后约 0.53 m。
        // 按用户要求从 258 帧恢复为 260 帧；最终脚前位置仍由近球
        // 视觉闭环完成，不再靠整数帧盲调。
        static constexpr int longWalkCommandFrames = 260;
        static constexpr int walkSettleFrames = 22;
        static constexpr double nominalLongWalkDistance = 3.354102;
        // 从理论路线航向到进攻中心航向还需左转 63.434949°；
        // 实际命令会闭环到下方的绝对 90° 目标并补偿长走残差。
        static constexpr double nominalBallFacingTurn = 63.434949;
        // 红方从固定初始朝向到进攻球门方向的绝对总左转角。
        // 第二转必须闭环到这个绝对 IMU 目标，不能把长走残余航向
        // 误差叠加到固定的 63.434949° 相对转角上。
        static constexpr double nominalAttackTurnFromStart =
            nominalInitialLeftTurn + nominalBallFacingTurn;
        static constexpr double walkHeadingFilterAlpha = 0.15;
        static constexpr double walkHeadingDeadZone = 1.5;
        static constexpr double walkHeadingGain = 0.20;
        static constexpr double maxWalkTurnPerCycle = 2.5;
        // 近球几何会直接决定后续落脚点，确认门槛高于远球搜索：不仅要
        // 连续检测到，还要求圆心和半径逐帧一致，最终使用多帧平均值。
        static constexpr int reacquireConfirmFrames = 5;
        static constexpr int reacquireMissFrameLimit = 12;
        // 260 帧固定路线结束时足球已经是近景目标。小于该半径的圆来自
        // 球下支撑/草地纹理，不具备作为脚前定位对象的物理尺度。
        static constexpr int nearBallMinimumRadiusPixels = 30;
        static constexpr int t4MinimumRadiusPixels = 24;
        // 固定使用右脚。脚中心距身体中心约一个足球半径，因此以
        // “球半径”为尺度定义脚前横向窗口，而不是使用随距离失效的
        // 固定像素偏置。图像右侧对应机器人右侧。
        static constexpr double rightFootTargetOffsetBallRadii = 1.0;
        static constexpr double footWindowToleranceBallRadii = 0.25;
        static constexpr int minimumFootWindowTolerancePixels = 8;
        static constexpr int coarseLateralCycleCount = 2;
        static constexpr int coarseMotionCommandFrames = 15;
        static constexpr int coarseMotionSettleFrames = 22;
        static constexpr int maxCoarseLateralPulses = 2;
        static constexpr double coarseYawAcceptDegrees = 1.5;
        static constexpr int maxCoarseYawCorrectionPulses = 2;
        static constexpr double maxCoarseYawCorrectionPerCycle = 4.0;
        // T6a 只测量一次保守前进脉冲的方向与图像增益。运动、停稳、
        // 航向恢复和静止重捕获严格串行，标定完成后没有后继动作。
        static constexpr double forwardCalibrationStep = 0.015;
        static constexpr int forwardCalibrationCycleCount = 2;
        static constexpr int forwardCalibrationCommandFrames = 15;
        static constexpr int forwardCalibrationSettleFrames = 22;
        static constexpr int maxForwardYawCorrectionPulses = 2;
        // 右脚与球门目标线存在横向偏置，0.6° 是有限的几何初值；
        // 它只改变 T3 目标 yaw，真实出球偏角仍需首次触球实测。
        static constexpr double rightFootKickAimCompensationDegrees = 0.6;
        // 以下精调距离仍是待触球验证的物理估计，不视为成功实测。
        // 上轮 radius≈50 时球仍在脚尖之外。640px / 1.3613rad 相机
        // 的焦距约 394px，球半径 0.07m，r=50 对应球心约 0.55m；
        // right_kick 的足心最大前伸 0.17m，碰撞足长 0.18m，
        // 再计入球半径 0.07m，理论触球边界约 0.33m。留出相机
        // 与身体原点的未知偏移，暂用 r=90~100 的更近窗口；
        // y 随头俯仰改变，只用于确认球完整留在画面内。
        static constexpr int kickBallRadiusMin = 90;
        static constexpr int kickBallRadiusMax = 100;
        static constexpr int kickBallRadiusSafetyMax = 108;
        static constexpr double kickObservationHeadPitch = 45.0;
        static constexpr double kickCloseHeadPitch = 60.0;
        static constexpr int fineMotionCycleCount = 2;
        static constexpr int fineMotionCommandFrames = 15;
        static constexpr int fineMotionSettleFrames = 18;
        static constexpr int maxFineForwardPulses = 10;
        static constexpr int maxFineLateralPulses = 5;
        static constexpr double fineYawAcceptDegrees = 1.0;
        // 每批前进/横移后的航向恢复各有自己的修正预算；不能把
        // 整段接近共用两次，否则第三批前进会在横向对正前被截断。
        static constexpr int maxFineYawCorrectionPulses = 3;
        static constexpr int kickRequestFrames = 5;
        static constexpr int kickWaitFrames = 38;
        static constexpr int settledLossConfirmFrames = 10;
        static constexpr int absoluteLossFrameLimit = 40;
        static constexpr int goalConfirmFrames = 5;
        static constexpr int keeperConfirmFrames = 3;
        // 第二次大转向的初始命令只是估计值。停稳后必须用 IMU 闭环
        // 补偿；超过次数仍不能对准则停下，不能带着大航向误差进入 T1。
        static constexpr double ballFacingYawAcceptDegrees = 3.0;
        static constexpr int maxBallFacingCorrectionPulses = 3;
        static constexpr double maxBallFacingCorrectionPerCycle = 8.0;
        static constexpr int shotSideConfirmFrames = 3;
        static constexpr int shotDecisionMaxWaitFrames = 8;
        static constexpr double keeperCenterDeadbandU = 0.08;
        static constexpr int defaultShotSide = -1;
        // 不瞄准门柱本身：14%/86% 对应在 2.6 m 球门内各留约
        // 0.36 m 安全余量。真实踢球测试后再决定是否继续外移。
        static constexpr double leftShotTargetU = 0.14;
        static constexpr double rightShotTargetU = 0.86;
        static constexpr double cameraHorizontalFieldOfView = 1.3613;
        static constexpr double radiansToDegrees = 57.29577951308232;
        static constexpr double shotTargetYawAcceptDegrees = 1.5;
        static constexpr double maxShotTargetVisualAngleDegrees = 20.0;
        static constexpr int maxShotTargetCorrectionPulses = 3;
        static constexpr double maxShotTargetCorrectionPerCycle = 4.0;

        static const auto normalizeAngle = [](double angle) {
            while (angle > 180.0) {
                angle -= 360.0;
            }
            while (angle <= -180.0) {
                angle += 360.0;
            }
            return angle;
        };

        // 仿真倍率并不固定。记录真实收到的相机消息序号，让视觉确认、
        // 转向保持和长距离行走都只随 Webots 新画面推进。
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
        static cv::Mat lastResultImage;
        const unsigned long long newCameraFrameCount =
            imageFrameSequence - lastProcessedImageFrame;
        const bool newCameraFrame = newCameraFrameCount > 0;
        if (newCameraFrame) {
            lastProcessedImageFrame = imageFrameSequence;
        }
        const int motionFrameAdvance = static_cast<int>(
            std::min<unsigned long long>(newCameraFrameCount, 100ULL)
        );

        // 未开始、暂停或结束时，motion 节点会自动让机器人保持准备姿态。
        // 每一轮先清除上一轮可能残留的运动命令。
        btask.type = btask.TASK_WALK;
        btask.step = 0.0;
        btask.lateral = 0.0;
        btask.turn = 0.0;
        btask.count = 0;
        const bool robotUpright = imuData.fall == imuData.FALL_NONE;
        bool enteredLongWalkThisFrame = false;
        bool enteredBallFacingTurnThisFrame = false;
        bool enteredShotTargetTurnThisFrame = false;

        htask.yaw = headTargetYaw;
        htask.pitch = headTargetPitch;

        // 只有 INIT 才代表监督程序已把机器人恢复到固定开球点。
        // PAUSE 中绝不能覆盖初始航向，否则恢复 PLAY 会重复整条路线。
        if (gameData.state == gameData.STATE_INIT) {
            initYaw = static_cast<double>(imuData.yaw);
            initYawReady = true;
        }

        // 只有真正的 INIT 才重置整条路线；READY/PAUSE/END 只会由默认
        // 的全零 BodyTask 停止动作，并保留当前进度供恢复使用。
        if (gameData.state != previousGameState) {
            if (gameData.state == gameData.STATE_INIT) {
                hasBallTrack = false;
                lostBallFrames = 0;
                settledLostBallFrames = 0;
                stableBallFrames = 0;
                liveBallFrames = 0;
                localReacquirePoseIndex = 0;
                headTargetYaw = 0.0;
                headTargetPitch = 20.0;
                approachPhase = ApproachPhase::WAIT_INITIAL_TURN;
                approachPhaseFrames = 0;
                reacquireAfterTurn = false;
                reacquireMatchFrames = 0;
                reacquireMissFrames = 0;
                goalViewStableFrames = 0;
                goalViewLocationSamples = 0;
                goalViewLocationXSum = 0.0;
                goalViewLocationZSum = 0.0;
                playStartRemainTime = -1;
                longWalkStartRemainTime = -1;
                routeCompleteRemainTime = -1;
                hasGoalTrack = false;
                trackedGoalRect = cv::Rect();
                goalStableFrames = 0;
                goalMissFrames = 0;
                hasKeeperTrack = false;
                trackedKeeperRect = cv::Rect();
                keeperStableFrames = 0;
                keeperMissFrames = 0;
                t1KeeperAnchorVisible = false;
                t1KeeperAnchorRect = cv::Rect();
                t1VerticalLineCandidates = 0;
                t1GoalDetectionMethod = 0;
                keeperMotionReady = false;
                filteredKeeperU = 0.5;
                leftShotClearance = 0.0;
                rightShotClearance = 0.0;
                keeperMotionSamples = 0;
                pendingShotSide = 0;
                shotSideStableFrames = 0;
                lockedShotSide = 0;
                hasFrozenShotGoal = false;
                frozenShotGoalRect = cv::Rect();
                shotDecisionWaitFrames = 0;
                shotTargetVisualAngleDegrees = 0.0;
                shotTargetAimAngleDegrees = 0.0;
                shotTargetImuYaw = initYaw;
                shotTargetPhysicalError = 0.0;
                shotTargetCorrectionPulses = 0;
                shotTargetAlignmentSafetyStop = false;
                ballReacquireSucceeded = false;
                ballReacquireSafetyStop = false;
                ballReacquireHeadYaw = 0.0;
                ballReacquireHeadPitch = 0.0;
                nearBallGeometrySamples = 0;
                nearBallCenterXSum = 0.0;
                nearBallCenterYSum = 0.0;
                nearBallRadiusSum = 0.0;
                nearBallConfirmedRadius = 0;
                coarseLateralCommand = 0.0;
                coarseInitialBallX = -1;
                coarseBeforeBallX = -1;
                coarseBeforeTargetX = -1;
                coarseAfterBallX = -1;
                coarsePixelShift = 0;
                preferredFootTargetX = -1;
                preferredFootTolerancePixels = 0;
                coarseLateralPulses = 0;
                coarseHeadingError = 0.0;
                coarseYawCorrectionPulses = 0;
                coarseDirectionMatched = false;
                coarseAlignmentReached = false;
                coarseSafetyStop = false;
                forwardCalibrationStarted = false;
                forwardReacquireSucceeded = false;
                forwardSafetyStop = false;
                forwardBeforeBallX = -1;
                forwardBeforeBallY = -1;
                forwardBeforeRadius = 0;
                forwardAfterBallX = -1;
                forwardAfterBallY = -1;
                forwardAfterRadius = 0;
                forwardDeltaBallY = 0;
                forwardDeltaRadius = 0;
                forwardBeforeHeadingError = 0.0;
                forwardAfterHeadingError = 0.0;
                forwardYawCorrectionPulses = 0;
                fineStepCommand = 0.0;
                fineLateralCommand = 0.0;
                fineForwardPulses = 0;
                fineLateralPulses = 0;
                fineYawCorrectionPulses = 0;
                fineSafetyStop = false;
                fineStopReason = "unknown";
                finalBallX = -1;
                finalBallY = -1;
                finalBallRadius = 0;
                finalHeadingError = 0.0;
                kickIssued = false;
                kickWaitCompleted = false;
                kickFallObserved = false;
                kickStartRemainTime = -1;
                kickStopRemainTime = -1;
                activeTurnCommand = 0.0;
                turnPulseStartYaw = initYaw;
                targetImuYaw = initYaw;
                imuYawDirectionForPositiveTurn = 1.0;
                imuTurnDirectionKnown = false;
                turnPulseNumber = 0;
                initialGeometryTurnComplete = false;
                initialTurnSafetyStop = false;
                latestPhysicalTurnError = nominalInitialLeftTurn;
                routeTravelStarted = false;
                routeTravelComplete = false;
                longWalkProcessedFrames = 0;
                filteredWalkHeadingError = 0.0;
                walkTurnCommand = 0.0;
                finalWalkHeadingError = 0.0;
                ballFacingTurnComplete = false;
                ballFacingTargetImuYaw = initYaw;
                ballFacingPhysicalError = 0.0;
                ballFacingCorrectionPulses = 0;
                ballFacingAlignmentSafetyStop = false;
                htask.yaw = headTargetYaw;
                htask.pitch = headTargetPitch;
            } else if (gameData.state == gameData.STATE_PLAY &&
                approachPhase == ApproachPhase::WAIT_INITIAL_TURN) {
                // 红方进入 PLAY 后直接按已知开球几何开始左转。第一次脉冲
                // 同时用于自动判断“左转”在 IMU yaw 上对应增大还是减小。
                if (!initYawReady) {
                    initYaw = static_cast<double>(imuData.yaw);
                    initYawReady = true;
                }
                headTargetYaw = 0.0;
                headTargetPitch = 20.0;
                htask.yaw = headTargetYaw;
                htask.pitch = headTargetPitch;
                playStartRemainTime = gameData.remain_time;
                if (myColor == COLOR_RED) {
                    // count=2 时每个完整步态周期使用相同转角，故一次性命令
                    // 取理论总角的一半，完成约 26.565° 的初始左转。
                    activeTurnCommand =
                        nominalInitialLeftTurn /
                        static_cast<double>(turnCycleCount);
                    turnPulseStartYaw = static_cast<double>(imuData.yaw);
                    turnPulseNumber = 1;
                    approachPhase = ApproachPhase::BODY_TURN;
                    approachPhaseFrames = turnCommandFrames;
                } else {
                    // 当前固定几何只对应用户正在测试的红方起点。
                    approachPhase = ApproachPhase::OBSERVE;
                    initialGeometryTurnComplete = true;
                    reacquireAfterTurn = true;
                }
            } else if ((gameData.state == gameData.STATE_PAUSE ||
                gameData.state == gameData.STATE_END) && kickIssued &&
                (approachPhase == ApproachPhase::KICK_REQUEST ||
                 approachPhase == ApproachPhase::KICK_WAIT)) {
                // 进球或出界会由 gamectrl 立即切到 PAUSE。将一次性踢球
                // 正常收口，避免永远冻结在 KICK_WAIT；比分用于区分结果。
                kickWaitCompleted = true;
                kickStopRemainTime = gameData.remain_time;
                approachPhase = ApproachPhase::KICK_STOP;
            }
            previousGameState = gameData.state;
        }

        // 连续行走时使用目标 IMU 航向闭环。误差采用身体左转为正的符号，
        // 再做低通和限幅；滤波只在收到新仿真画面时推进一次。
        if (gameData.state == gameData.STATE_PLAY &&
            robotUpright &&
            approachPhase == ApproachPhase::LONG_WALK) {
            double physicalHeadingError = 0.0;
            if (imuTurnDirectionKnown) {
                physicalHeadingError = normalizeAngle(
                    targetImuYaw - static_cast<double>(imuData.yaw)
                ) * imuYawDirectionForPositiveTurn;
            }
            if (newCameraFrame) {
                const double effectiveFilterAlpha = 1.0 - std::pow(
                    1.0 - walkHeadingFilterAlpha,
                    static_cast<double>(motionFrameAdvance)
                );
                filteredWalkHeadingError =
                    (1.0 - effectiveFilterAlpha) *
                        filteredWalkHeadingError +
                    effectiveFilterAlpha * physicalHeadingError;
            }
            walkTurnCommand = 0.0;
            if (std::abs(filteredWalkHeadingError) > walkHeadingDeadZone) {
                walkTurnCommand = std::max(
                    -maxWalkTurnPerCycle,
                    std::min(
                        maxWalkTurnPerCycle,
                        walkHeadingGain * filteredWalkHeadingError
                    )
                );
            }
        }

        // 转向与连续前进都通过独立状态发布，其他循环保持全零，
        // 防止上一阶段的运动命令残留。
        if (gameData.state == gameData.STATE_PLAY &&
            robotUpright &&
            (approachPhase == ApproachPhase::BODY_TURN ||
             approachPhase == ApproachPhase::BALL_FACING_TURN ||
             approachPhase == ApproachPhase::SHOT_TARGET_TURN ||
             approachPhase == ApproachPhase::COARSE_YAW_RECOVER ||
             approachPhase == ApproachPhase::FORWARD_YAW_RECOVER ||
             approachPhase == ApproachPhase::FINE_YAW_RECOVER)) {
            btask.step = 0.0;
            btask.lateral = 0.0;
            btask.turn = activeTurnCommand;
            btask.count = turnCycleCount;
        } else if (gameData.state == gameData.STATE_PLAY &&
            robotUpright &&
            approachPhase == ApproachPhase::LONG_WALK) {
            btask.step = longWalkStep;
            btask.lateral = 0.0;
            btask.turn = walkTurnCommand;
            btask.count = longWalkCycleCount;
        } else if (gameData.state == gameData.STATE_PLAY &&
            robotUpright &&
            approachPhase == ApproachPhase::COARSE_LATERAL_MOVE) {
            btask.step = 0.0;
            btask.lateral = coarseLateralCommand;
            btask.turn = 0.0;
            btask.count = coarseLateralCycleCount;
        } else if (gameData.state == gameData.STATE_PLAY &&
            robotUpright &&
            approachPhase == ApproachPhase::FORWARD_CALIBRATION_MOVE) {
            btask.step = forwardCalibrationStep;
            btask.lateral = 0.0;
            btask.turn = 0.0;
            btask.count = forwardCalibrationCycleCount;
        } else if (gameData.state == gameData.STATE_PLAY &&
            robotUpright && approachPhase == ApproachPhase::FINE_MOVE) {
            btask.step = fineStepCommand;
            btask.lateral = fineLateralCommand;
            btask.turn = 0.0;
            btask.count = fineMotionCycleCount;
        } else if (gameData.state == gameData.STATE_PLAY &&
            robotUpright && approachPhase == ApproachPhase::KICK_REQUEST) {
            btask.type = btask.TASK_ACT;
            btask.actname = "right_kick";
            btask.step = 0.0;
            btask.lateral = 0.0;
            btask.turn = 0.0;
            btask.count = 1;
        }

        // 身体转向、连续行走和停稳时，相机图像包含周期性摆动与运动模糊。
        // 所有运动阶段完全暂停视觉，也不绘制旧球框；回到 OBSERVE 后
        // 清空旧跟踪，并只用身体静止后的新画面重新识别。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright &&
            !image.empty() &&
            (approachPhase == ApproachPhase::BALL_REACQUIRE ||
             approachPhase == ApproachPhase::COARSE_BALL_REACQUIRE ||
             approachPhase == ApproachPhase::FORWARD_BALL_REACQUIRE ||
             approachPhase == ApproachPhase::FINE_BALL_REACQUIRE)) {
            // 远距离足球实测只有约 16~20 像素。当前保留原分辨率，避免
            // 下采样后被形态学核抹掉；视觉只在静止阶段运行，开销可控。
            cv::Mat small;
            cv::resize(image, small, cv::Size(), 1.0, 1.0, cv::INTER_AREA);

            // 输入图像是 RGB，因此使用 RGB2HSV。
            cv::Mat hsv;
            cv::cvtColor(small, hsv, cv::COLOR_RGB2HSV);

            // 初步提取高亮、低饱和度的白色区域。
            cv::Mat hsvWhiteMask;
            cv::inRange(
                hsv,
                cv::Scalar(0, 0, 145),
                cv::Scalar(180, 85, 255),
                hsvWhiteMask
            );

            // Lab 的 L 通道表示亮度，a/b 通道接近中性值时更像白色。
            // 与 HSV 掩膜取并集，避免光照或颜色偏移导致足球白色区域漏检。
            cv::Mat lab;
            cv::cvtColor(small, lab, cv::COLOR_RGB2Lab);
            cv::Mat labWhiteMask;
            cv::inRange(
                lab,
                cv::Scalar(140, 105, 105),
                cv::Scalar(255, 150, 150),
                labWhiteMask
            );

            cv::Mat whiteMask;
            cv::bitwise_or(hsvWhiteMask, labWhiteMask, whiteMask);

            // 足球的深色花纹通常是低饱和度的暗区域，草地则具有较高饱和度。
            // 将这类暗区域并入轮廓掩膜，有助于把足球整体从白线中分离出来。
            cv::Mat darkNeutralMask;
            cv::inRange(
                hsv,
                cv::Scalar(0, 0, 0),
                cv::Scalar(180, 110, 110),
                darkNeutralMask
            );

            // 草地通常是高饱和度的绿色。这个掩膜只用于候选评分，
            // 不直接参与轮廓生成，避免误删足球上的彩色面片。
            cv::Mat greenMask;
            cv::inRange(
                hsv,
                cv::Scalar(25, 50, 30),
                cv::Scalar(95, 255, 255),
                greenMask
            );

            cv::Mat ballMask;
            cv::bitwise_or(whiteMask, darkNeutralMask, ballMask);

            // 填补小缺口，减少足球白色区域被黑色花纹分割的影响。
            cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(5, 5)
            );
            cv::morphologyEx(
                whiteMask,
                whiteMask,
                cv::MORPH_CLOSE,
                kernel
            );

            // 轮廓掩膜先做轻度开运算，尽量断开场地细白线与足球的连接，
            // 再闭运算和膨胀连接足球上相邻但分散的白色块。
            // 原始 whiteMask 仍用于覆盖率和白色比例计算，避免形态学操作改变评分。
            cv::Mat contourMask;
            cv::Mat separatedMask;
            cv::Mat separationKernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(7, 7)
            );
            cv::Mat contourKernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(3, 3)
            );
            cv::morphologyEx(
                ballMask,
                separatedMask,
                cv::MORPH_OPEN,
                separationKernel
            );
            cv::morphologyEx(
                separatedMask,
                separatedMask,
                cv::MORPH_CLOSE,
                kernel
            );
            cv::dilate(separatedMask, contourMask, contourKernel);

            std::vector<std::vector<cv::Point>> contours;
            // 远球轮廓链已退出红方固定开局主路径。保留空容器只为让
            // 下方历史评分代码零成本跳过；近球统一使用全图 Hough。
            (void)contourMask;

            // 足球的黑色花纹可能被场地白线连成一个大轮廓，
            // 因此额外从黑色小区域建立局部候选框。
            cv::Mat darkSeedMask;
            cv::Mat darkSeedKernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(3, 3)
            );
            cv::morphologyEx(
                darkNeutralMask,
                darkSeedMask,
                cv::MORPH_OPEN,
                darkSeedKernel
            );
            std::vector<std::vector<cv::Point>> darkContours;
            (void)darkSeedMask;

            struct CandidateProposal {
                cv::Rect rect;
                cv::Point center;
                double area;
                double perimeter;
                bool fromDarkSeed;
            };

            std::vector<CandidateProposal> proposals;
            proposals.reserve(contours.size() + darkContours.size());
            for (const auto &contour : contours) {
                double area = cv::contourArea(contour);
                if (area < 1.0) {
                    continue;
                }
                cv::Rect contourRect = cv::boundingRect(contour);
                proposals.push_back({
                    contourRect,
                    cv::Point(
                        contourRect.x + contourRect.width / 2,
                        contourRect.y + contourRect.height / 2
                    ),
                    area,
                    cv::arcLength(contour, true),
                    false
                });
            }

            for (const auto &contour : darkContours) {
                double area = cv::contourArea(contour);
                cv::Rect seedRect = cv::boundingRect(contour);
                int seedSize = std::max(seedRect.width, seedRect.height);

                // 过滤单像素噪声和明显过大的黑色区域。
                if (area < 2.0 || seedSize < 2 || seedSize > small.cols * 0.12) {
                    continue;
                }

                cv::Point seedCenter(
                    seedRect.x + seedRect.width / 2,
                    seedRect.y + seedRect.height / 2
                );
                int proposalSize = std::max(14, seedSize * 4);
                proposalSize = std::min(
                    proposalSize,
                    static_cast<int>(small.cols * 0.22)
                );
                cv::Rect proposalRect(
                    seedCenter.x - proposalSize / 2,
                    seedCenter.y - proposalSize / 2,
                    proposalSize,
                    proposalSize
                );
                proposalRect &= cv::Rect(0, 0, small.cols, small.rows);
                if (proposalRect.width < 8 || proposalRect.height < 8) {
                    continue;
                }

                proposals.push_back({
                    proposalRect,
                    seedCenter,
                    area,
                    cv::arcLength(contour, true),
                    true
                });
            }

            bool foundBallCandidate = false;
            cv::Rect bestRect;
            cv::Point bestCenter;
            double bestScore = 0.0;
            bool nearBallCircleRecovered = false;
            int nearCircleProposalCount = 0;
            int nearCircleAcceptedCount = 0;
            const double maxTrackJump = small.cols * 0.18;
            const bool headAtLastTarget =
                std::abs(static_cast<double>(headAngle.yaw) - headTargetYaw) <= 3.0 &&
                std::abs(static_cast<double>(headAngle.pitch) - headTargetPitch) <= 3.0;

            for (const auto &proposal : proposals) {
                double area = proposal.area;
                cv::Rect rect = proposal.rect;

                // 太细长的区域更可能是场地线。
                if (rect.width < 2 || rect.height < 2) {
                    continue;
                }

                if (rect.width > rect.height * 3.0 ||
                    rect.height > rect.width * 3.0) {
                    continue;
                }

                // 排除贴近图像边缘且被截断的候选，避免无法估计完整形状。
                // 这是通用的成像边界保护，不依赖某个固定视角的区域内容。
                const int edgeMargin = std::max(3, static_cast<int>(small.cols * 0.05));
                if (rect.x <= edgeMargin || rect.y <= edgeMargin ||
                    rect.x + rect.width >= small.cols - edgeMargin ||
                    rect.y + rect.height >= small.rows - edgeMargin) {
                    continue;
                }

                // 足球通常是局部小目标，过大的区域更可能是白线或球门。
                if (rect.width > small.cols * 0.25 ||
                    rect.height > small.rows * 0.25) {
                    continue;
                }

                double perimeter = proposal.perimeter;
                if (perimeter <= 0.0) {
                    continue;
                }

                int longSide = std::max(rect.width, rect.height);
                int shortSide = std::min(rect.width, rect.height);

                if (longSide <= 0) {
                    continue;
                }

                double aspect = static_cast<double>(shortSide) / longSide;
                double circularity =
                    4.0 * CV_PI * area / (perimeter * perimeter);

                // 足球的黑色花纹会把白色区域分割成小块，
                // 因此不能要求单个白色轮廓具有很高的圆度。
                if (!proposal.fromDarkSeed &&
                    (aspect < 0.20 || circularity < 0.04)) {
                    continue;
                }

                // 统计候选框中白色像素的比例。
                // 足球白色区域超过一半，但不会像纯白场地线那样接近 100%。
                double whiteRatio = static_cast<double>(
                    cv::countNonZero(whiteMask(rect))) /
                    static_cast<double>(rect.area());
                // 规则要求足球白色区域超过一半；考虑掩膜漏检，
                // 这里保留一定余量，后续再用白色比例进行评分。
                if (whiteRatio < 0.35 || whiteRatio > 0.92) {
                    continue;
                }

                // 足球白色区域之间通常能看到黑色或深色花纹，
                // 纯白场地线在候选框内则几乎没有深色像素。
                double darkRatio = static_cast<double>(cv::countNonZero(darkNeutralMask(rect))) /
                    static_cast<double>(rect.area());
                if (darkRatio < 0.05) {
                    continue;
                }

                double greenRatio = static_cast<double>(cv::countNonZero(greenMask(rect))) /
                    static_cast<double>(rect.area());
                if (greenRatio > 0.78) {
                    continue;
                }

                // 对黑白联合区域做一次局部闭运算，估计候选的真实紧凑程度。
                // 黑色种子候选本身是人为扩展的方框，不能直接拿方框形状评分。
                cv::Mat localObjectMask = ballMask(rect).clone();
                cv::Mat objectKernel = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE,
                    cv::Size(5, 5)
                );
                cv::morphologyEx(
                    localObjectMask,
                    localObjectMask,
                    cv::MORPH_CLOSE,
                    objectKernel
                );
                std::vector<std::vector<cv::Point>> objectContours;
                cv::findContours(
                    localObjectMask,
                    objectContours,
                    cv::RETR_EXTERNAL,
                    cv::CHAIN_APPROX_SIMPLE
                );
                cv::Point localCenter(
                    proposal.center.x - rect.x,
                    proposal.center.y - rect.y
                );
                double objectArea = 0.0;
                double objectPerimeter = 0.0;
                cv::Rect objectRect;
                for (const auto &contour : objectContours) {
                    cv::Rect contourRect = cv::boundingRect(contour);
                    double contourArea = cv::contourArea(contour);
                    if (contourArea > objectArea && contourRect.contains(localCenter)) {
                        objectArea = contourArea;
                        objectPerimeter = cv::arcLength(contour, true);
                        objectRect = contourRect;
                    }
                }
                if (objectArea <= 0.0 || objectPerimeter <= 0.0) {
                    continue;
                }
                int objectLongSide = std::max(objectRect.width, objectRect.height);
                int objectShortSide = std::min(objectRect.width, objectRect.height);
                if (objectLongSide <= 0 ||
                    static_cast<double>(objectShortSide) / objectLongSide < 0.35) {
                    continue;
                }
                double objectCircularity =
                    4.0 * CV_PI * objectArea / (objectPerimeter * objectPerimeter);
                double objectCoverage = objectArea / static_cast<double>(rect.area());
                if (objectCoverage < 0.10) {
                    continue;
                }

                // 场地线的白色像素通常集中在单独的一行或一列，
                // 足球的白色区域则会分布在候选框内部多个位置。
                cv::Mat candidateWhite = whiteMask(rect);
                int maxWhiteRow = 0;
                int maxWhiteColumn = 0;
                for (int row = 0; row < candidateWhite.rows; ++row) {
                    maxWhiteRow = std::max(
                        maxWhiteRow,
                        cv::countNonZero(candidateWhite.row(row))
                    );
                }
                for (int column = 0; column < candidateWhite.cols; ++column) {
                    maxWhiteColumn = std::max(
                        maxWhiteColumn,
                        cv::countNonZero(candidateWhite.col(column))
                    );
                }
                double rowConcentration = static_cast<double>(maxWhiteRow) /
                    static_cast<double>(rect.width);
                double columnConcentration = static_cast<double>(maxWhiteColumn) /
                    static_cast<double>(rect.height);
                double lineConcentration = std::max(
                    rowConcentration,
                    columnConcentration
                );
                if (lineConcentration > 0.97 && whiteRatio > 0.20) {
                    continue;
                }

                // 面积只做有限贡献，避免大白线压过小足球。
                double areaScore = std::min(area / 300.0, 1.0);

                // 白色比例接近 0.6 时更像有黑色花纹的足球。
                double patternScore =
                    1.0 - std::min(std::abs(whiteRatio - 0.60), 0.60);

                // 黑色种子候选的几何形状来自局部窗口，
                // 用窗口内的草地比例抑制黑色草地阴影误检。
                double seedScore = proposal.fromDarkSeed ? 1.0 : 0.0;
                double greenScore = 1.0 - std::min(greenRatio / 0.78, 1.0);
                double compactScore = std::min(objectCircularity / 0.60, 1.0);

                double score =
                    3.0 * aspect +
                    3.0 * circularity +
                    3.0 * patternScore +
                    2.0 * std::min(darkRatio / 0.25, 1.0) +
                    2.0 * greenScore +
                    2.0 * compactScore +
                    1.5 * std::min(objectCoverage / 0.60, 1.0) +
                    2.0 * seedScore +
                    3.0 * (1.0 - std::min(lineConcentration, 1.0)) +
                    areaScore;

                // 相邻帧的目标位置应连续变化，时间约束只用于抑制单帧跳变，
                // 不对足球在视野中的绝对位置做假设。
                if (hasBallTrack) {
                    double dx = static_cast<double>(proposal.center.x - trackedBallCenter.x);
                    double dy = static_cast<double>(proposal.center.y - trackedBallCenter.y);
                    double trackDistance = std::sqrt(dx * dx + dy * dy);
                    if (trackDistance > maxTrackJump) {
                        continue;
                    }
                    score += 4.0 * (1.0 - trackDistance / maxTrackJump);
                }

                if (!foundBallCandidate || score > bestScore) {
                    foundBallCandidate = true;
                    bestScore = score;
                    bestRect = rect;
                    bestCenter = proposal.center;
                }
            }

            // T4 静止近球重捕获使用已验证的完整圆几何；T5 沿用
            // 原有同头姿重捕获链。T6 及后续近球处理暂时保持不变。
            const bool closeBallGeometryRequired =
                approachPhase == ApproachPhase::BALL_REACQUIRE ||
                approachPhase == ApproachPhase::FORWARD_BALL_REACQUIRE ||
                approachPhase == ApproachPhase::FINE_BALL_REACQUIRE;
            if (closeBallGeometryRequired) {
                foundBallCandidate = false;
                const bool t4Observation =
                    approachPhase == ApproachPhase::BALL_REACQUIRE;
                const int minimumRadiusPixels = t4Observation
                    ? t4MinimumRadiusPixels
                    : nearBallMinimumRadiusPixels;
                cv::Mat circleGray;
                cv::cvtColor(small, circleGray, cv::COLOR_RGB2GRAY);
                cv::GaussianBlur(
                    circleGray,
                    circleGray,
                    cv::Size(9, 9),
                    2.0,
                    2.0
                );
                cv::Mat circleEdges;
                cv::Canny(circleGray, circleEdges, 55.0, 130.0);

                std::vector<cv::Vec3f> circles;
                cv::HoughCircles(
                    circleGray,
                    circles,
                    cv::HOUGH_GRADIENT,
                    1.2,
                    24.0,
                    100.0,
                    14.0,
                    minimumRadiusPixels,
                    std::min(
                        t4Observation ? 90 : 110,
                        std::min(small.cols, small.rows) / 4
                    )
                );
                nearCircleProposalCount = static_cast<int>(circles.size());

                double bestCircleScore = -1.0;
                int bestCircleRadius = 0;
                for (const cv::Vec3f &circle : circles) {
                    const cv::Point center(
                        cvRound(circle[0]),
                        cvRound(circle[1])
                    );
                    const int radius = cvRound(circle[2]);
                    const int outerRadius = cvRound(radius * 1.45);
                    const int requiredMargin = t4Observation
                        ? outerRadius : radius;
                    if (radius < minimumRadiusPixels ||
                        center.x - requiredMargin < 2 ||
                        center.y - requiredMargin < 2 ||
                        center.x + requiredMargin >= small.cols - 2 ||
                        center.y + requiredMargin >= small.rows - 2) {
                        continue;
                    }

                    const cv::Rect circleRect = cv::Rect(
                        center.x - outerRadius,
                        center.y - outerRadius,
                        outerRadius * 2 + 1,
                        outerRadius * 2 + 1
                    ) & cv::Rect(0, 0, small.cols, small.rows);
                    const cv::Point localCenter = center - circleRect.tl();
                    cv::Mat interior = cv::Mat::zeros(
                        circleRect.size(),
                        CV_8UC1
                    );
                    cv::circle(
                        interior,
                        localCenter,
                        std::max(1, cvRound(radius * 0.88)),
                        cv::Scalar(255),
                        -1
                    );
                    const int interiorPixels = cv::countNonZero(interior);
                    if (interiorPixels <= 0) {
                        continue;
                    }

                    cv::Mat sampled;
                    cv::bitwise_and(
                        whiteMask(circleRect),
                        interior,
                        sampled
                    );
                    const double whiteRatio =
                        static_cast<double>(cv::countNonZero(sampled)) /
                        static_cast<double>(interiorPixels);
                    cv::bitwise_and(
                        darkNeutralMask(circleRect),
                        interior,
                        sampled
                    );
                    const double darkRatio =
                        static_cast<double>(cv::countNonZero(sampled)) /
                        static_cast<double>(interiorPixels);
                    cv::bitwise_and(
                        greenMask(circleRect),
                        interior,
                        sampled
                    );
                    const double greenRatio =
                        static_cast<double>(cv::countNonZero(sampled)) /
                        static_cast<double>(interiorPixels);
                    cv::Mat blueRobotMask;
                    cv::inRange(
                        hsv(circleRect),
                        cv::Scalar(95, 75, 40),
                        cv::Scalar(135, 255, 255),
                        blueRobotMask
                    );
                    cv::bitwise_and(
                        blueRobotMask,
                        interior,
                        sampled
                    );
                    const double blueRobotRatio =
                        static_cast<double>(cv::countNonZero(sampled)) /
                        static_cast<double>(interiorPixels);

                    cv::Mat surroundingRing = cv::Mat::zeros(
                        circleRect.size(),
                        CV_8UC1
                    );
                    cv::circle(
                        surroundingRing,
                        localCenter,
                        outerRadius,
                        cv::Scalar(255),
                        -1
                    );
                    cv::circle(
                        surroundingRing,
                        localCenter,
                        cvRound(radius * 1.08),
                        cv::Scalar(0),
                        -1
                    );
                    const int surroundingPixels =
                        cv::countNonZero(surroundingRing);
                    cv::bitwise_and(
                        greenMask(circleRect),
                        surroundingRing,
                        sampled
                    );
                    const double surroundingGreenRatio =
                        static_cast<double>(cv::countNonZero(sampled)) /
                        static_cast<double>(surroundingPixels);

                    cv::Mat edgeBand = cv::Mat::zeros(
                        circleRect.size(),
                        CV_8UC1
                    );
                    cv::circle(
                        edgeBand,
                        localCenter,
                        cvRound(radius * 1.12),
                        cv::Scalar(255),
                        -1
                    );
                    cv::circle(
                        edgeBand,
                        localCenter,
                        cvRound(radius * 0.82),
                        cv::Scalar(0),
                        -1
                    );
                    const int edgeBandPixels = cv::countNonZero(edgeBand);
                    cv::bitwise_and(
                        circleEdges(circleRect),
                        edgeBand,
                        sampled
                    );
                    const double edgeSupport =
                        static_cast<double>(cv::countNonZero(sampled)) /
                        static_cast<double>(edgeBandPixels);

                    if (whiteRatio < 0.38 ||
                        darkRatio < 0.05 ||
                        greenRatio > 0.42 ||
                        // 足球剩余面片可能是蓝色，不能一律排除蓝色；
                        // 仅在白色证据不足且蓝色占优时拒绝门将局部。
                        (whiteRatio < 0.50 &&
                        blueRobotRatio > 0.16) ||
                        surroundingGreenRatio < 0.28 ||
                        edgeSupport < 0.035) {
                        continue;
                    }

                    // 脚前复测不能只凭一小段白线边缘通过圆检测：
                    // 真球的外轮廓应分布在圆周多数方向。
                    if (approachPhase ==
                        ApproachPhase::FINE_BALL_REACQUIRE) {
                        int edgeSectors[8] = {};
                        std::vector<cv::Point> edgePoints;
                        cv::findNonZero(sampled, edgePoints);
                        for (const cv::Point &point : edgePoints) {
                            const double angle = std::atan2(
                                static_cast<double>(point.y - localCenter.y),
                                static_cast<double>(point.x - localCenter.x)
                            );
                            const int sector = std::max(
                                0,
                                std::min(
                                    7,
                                    static_cast<int>((angle +
                                        3.141592653589793) *
                                        4.0 / 3.141592653589793)
                                )
                            );
                            ++edgeSectors[sector];
                        }
                        int occupiedSectors = 0;
                        for (const int count : edgeSectors) {
                            if (count >= std::max(2, radius / 30)) {
                                ++occupiedSectors;
                            }
                        }
                        if (occupiedSectors < 5) {
                            continue;
                        }
                    }
                    ++nearCircleAcceptedCount;

                    double temporalScore = 0.0;
                    if (hasBallTrack && trackedBallRect.width > 0) {
                        const double dx = static_cast<double>(
                            center.x - trackedBallCenter.x
                        );
                        const double dy = static_cast<double>(
                            center.y - trackedBallCenter.y
                        );
                        const double distance = std::sqrt(dx * dx + dy * dy);
                        const double previousRadius =
                            trackedBallRect.width * 0.5;
                        const double allowedDistance = std::max(
                            8.0,
                            previousRadius * 0.35
                        );
                        if (distance > allowedDistance ||
                            std::abs(radius - previousRadius) >
                                std::max(5.0, previousRadius * 0.25)) {
                            continue;
                        }
                        temporalScore = 2.0 *
                            (1.0 - distance / allowedDistance);
                    }

                    const double circleScore =
                        whiteRatio * 3.0 +
                        darkRatio * 2.0 -
                        greenRatio * 3.0 -
                        blueRobotRatio * 2.0 +
                        surroundingGreenRatio * 3.0 +
                        edgeSupport * 12.0 +
                        temporalScore;
                    if (circleScore > bestCircleScore) {
                        bestCircleScore = circleScore;
                        bestCenter = center;
                        bestCircleRadius = radius;
                    }
                }

                if (bestCircleScore >= 0.0 && bestCircleRadius > 0) {
                    bestRect = cv::Rect(
                        bestCenter.x - bestCircleRadius,
                        bestCenter.y - bestCircleRadius,
                        bestCircleRadius * 2,
                        bestCircleRadius * 2
                    );
                    foundBallCandidate = true;
                    nearBallCircleRecovered = true;
                }
            }

            bool detectedThisFrame = foundBallCandidate;
            if (reacquireAfterTurn) {
                // 头部到位后的连续静止画面才能完成 T4。除了“连续看到”，
                // 还要求圆心与半径没有跳变；最终球心取整段观测的平均值，
                // 单帧 Hough 抖动不能直接变成落脚误差。
                if (detectedThisFrame && headAtLastTarget) {
                    const int detectedRadius = std::max(
                        bestRect.width,
                        bestRect.height
                    ) / 2;
                    bool geometryConsistent = true;
                    if (nearBallGeometrySamples > 0 &&
                        trackedBallRect.width > 0) {
                        const double dx = static_cast<double>(
                            bestCenter.x - trackedBallCenter.x
                        );
                        const double dy = static_cast<double>(
                            bestCenter.y - trackedBallCenter.y
                        );
                        const double previousRadius =
                            trackedBallRect.width * 0.5;
                        geometryConsistent =
                            std::sqrt(dx * dx + dy * dy) <=
                                std::max(8.0, previousRadius * 0.35) &&
                            std::abs(detectedRadius - previousRadius) <=
                                std::max(5.0, previousRadius * 0.25);
                    }
                    if (!geometryConsistent) {
                        nearBallGeometrySamples = 0;
                        nearBallCenterXSum = 0.0;
                        nearBallCenterYSum = 0.0;
                        nearBallRadiusSum = 0.0;
                    }
                    if (nearBallGeometrySamples < reacquireConfirmFrames) {
                        ++nearBallGeometrySamples;
                        nearBallCenterXSum += bestCenter.x;
                        nearBallCenterYSum += bestCenter.y;
                        nearBallRadiusSum += detectedRadius;
                    }
                    reacquireMatchFrames = nearBallGeometrySamples;
                    reacquireMissFrames = 0;
                } else {
                    reacquireMatchFrames = 0;
                    nearBallGeometrySamples = 0;
                    nearBallCenterXSum = 0.0;
                    nearBallCenterYSum = 0.0;
                    nearBallRadiusSum = 0.0;
                    // 头部运动中的模糊画面不能算作识别失败；只有实际角度
                    // 到达当前局部观察姿态后，才累计连续静止漏检帧。
                    if (headAtLastTarget) {
                        reacquireMissFrames = std::min(
                            reacquireMissFrames + 1,
                            reacquireMissFrameLimit
                        );
                    } else {
                        reacquireMissFrames = 0;
                    }
                }
            }
            if (foundBallCandidate) {
                if (approachPhase == ApproachPhase::OBSERVE &&
                    !reacquireAfterTurn) {
                    liveBallFrames = std::min(liveBallFrames + 1, 5);
                } else {
                    // 运动后的重新捕获缓冲只确认候选，不触发新的身体动作。
                    liveBallFrames = 0;
                }
                if (hasBallTrack) {
                    stableBallFrames = std::min(stableBallFrames + 1, 5);
                } else {
                    stableBallFrames = 1;
                }
                trackedBallCenter = bestCenter;
                trackedBallRect = bestRect;
                hasBallTrack = true;
                lostBallFrames = 0;
                settledLostBallFrames = 0;
            } else if (hasBallTrack && !reacquireAfterTurn &&
                lostBallFrames < absoluteLossFrameLimit &&
                (!headAtLastTarget ||
                 settledLostBallFrames < settledLossConfirmFrames)) {
                // 候选消失后保持最后目标。头部仍在转动时不开始“确实丢球”计时；
                // 只有头部到位、画面稳定后，才累计连续静止丢失帧。
                bestCenter = trackedBallCenter;
                bestRect = trackedBallRect;
                foundBallCandidate = true;
                ++lostBallFrames;
                if (headAtLastTarget) {
                    ++settledLostBallFrames;
                } else {
                    settledLostBallFrames = 0;
                }
                liveBallFrames = 0;
            } else {
                hasBallTrack = false;
                lostBallFrames = 0;
                settledLostBallFrames = 0;
                stableBallFrames = 0;
                liveBallFrames = 0;
            }

            if (foundBallCandidate && stableBallFrames >= 3) {
                cv::Rect displayRect(
                    bestRect.x,
                    bestRect.y,
                    bestRect.width,
                    bestRect.height
                );

                // 候选筛选使用的窗口可能是由黑色花纹向外扩展得到的，
                // 精修后的框同时用于显示和相对距离判断，保证两者语义一致。
                if (!nearBallCircleRecovered &&
                    approachPhase != ApproachPhase::BALL_REACQUIRE &&
                    approachPhase != ApproachPhase::FINE_BALL_REACQUIRE) {
                    cv::Mat localMask = ballMask(displayRect).clone();
                    cv::Mat refineKernel = cv::getStructuringElement(
                        cv::MORPH_ELLIPSE,
                        cv::Size(3, 3)
                    );
                    cv::morphologyEx(
                        localMask,
                        localMask,
                        cv::MORPH_CLOSE,
                        refineKernel
                    );
                    cv::dilate(localMask, localMask, refineKernel);

                    std::vector<std::vector<cv::Point>> localContours;
                    cv::findContours(
                        localMask,
                        localContours,
                        cv::RETR_EXTERNAL,
                        cv::CHAIN_APPROX_SIMPLE
                    );
                    cv::Point localCenter(
                        bestCenter.x - displayRect.x,
                        bestCenter.y - displayRect.y
                    );
                    double refinedArea = 0.0;
                    cv::Rect refinedRect;
                    for (const auto &contour : localContours) {
                        double area = cv::contourArea(contour);
                        cv::Rect contourRect = cv::boundingRect(contour);
                        if (area < refinedArea ||
                            !contourRect.contains(localCenter)) {
                            continue;
                        }
                        refinedArea = area;
                        refinedRect = contourRect;
                    }
                    if (refinedArea > 0.0 &&
                        refinedRect.width >= 4 &&
                        refinedRect.height >= 4) {
                        const int padding = 2;
                        refinedRect.x = std::max(0, refinedRect.x - padding);
                        refinedRect.y = std::max(0, refinedRect.y - padding);
                        refinedRect.width = std::min(
                            displayRect.width - refinedRect.x,
                            refinedRect.width + 2 * padding
                        );
                        refinedRect.height = std::min(
                            displayRect.height - refinedRect.y,
                            refinedRect.height + 2 * padding
                        );
                        displayRect.x += refinedRect.x;
                        displayRect.y += refinedRect.y;
                        displayRect.width = refinedRect.width;
                        displayRect.height = refinedRect.height;
                    }
                }

                const cv::Point measuredBallCenter(
                    displayRect.x + displayRect.width / 2,
                    displayRect.y + displayRect.height / 2
                );
                if ((approachPhase == ApproachPhase::BALL_REACQUIRE ||
                     approachPhase ==
                        ApproachPhase::FINE_BALL_REACQUIRE) &&
                    detectedThisFrame) {
                    trackedBallRect = displayRect;
                    trackedBallCenter = measuredBallCenter;
                }
                const int measuredBallSize = std::max(
                    displayRect.width,
                    displayRect.height
                );
                std::string approachStatus = "route: observing";
                if (approachPhase == ApproachPhase::BALL_REACQUIRE) {
                    approachStatus = "T4 ball reacquiring " +
                        std::to_string(reacquireMatchFrames) + "/" +
                        std::to_string(reacquireConfirmFrames) +
                        " circles=" +
                        std::to_string(nearCircleProposalCount) + "/" +
                        std::to_string(nearCircleAcceptedCount) +
                        " yaw=" + std::to_string(headTargetYaw) +
                        " pitch=" + std::to_string(headTargetPitch);
                } else if (approachPhase ==
                    ApproachPhase::COARSE_BALL_REACQUIRE) {
                    approachStatus = "T5 ball reacquiring " +
                        std::to_string(reacquireMatchFrames) + "/" +
                        std::to_string(reacquireConfirmFrames) +
                        " circles=" +
                        std::to_string(nearCircleProposalCount) + "/" +
                        std::to_string(nearCircleAcceptedCount);
                } else if (approachPhase ==
                    ApproachPhase::FORWARD_BALL_REACQUIRE) {
                    approachStatus = "T6 ball reacquiring " +
                        std::to_string(reacquireMatchFrames) + "/" +
                        std::to_string(reacquireConfirmFrames) +
                        " circles=" +
                        std::to_string(nearCircleProposalCount) + "/" +
                        std::to_string(nearCircleAcceptedCount);
                } else if (approachPhase ==
                    ApproachPhase::FINE_BALL_REACQUIRE) {
                    approachStatus = "shot-ready ball reacquiring " +
                        std::to_string(reacquireMatchFrames) + "/" +
                        std::to_string(reacquireConfirmFrames) +
                        " circles=" +
                        std::to_string(nearCircleProposalCount) + "/" +
                        std::to_string(nearCircleAcceptedCount);
                } else if (!detectedThisFrame) {
                    approachStatus = headAtLastTarget
                        ? "body: confirming lost"
                        : "body: waiting for scan head";
                } else if (!headAtLastTarget) {
                    approachStatus = "body: waiting for scan head";
                } else if (liveBallFrames < 3) {
                    approachStatus = "body: confirming " +
                        std::to_string(liveBallFrames) + "/3";
                } else if (routeTravelComplete) {
                    // 第一个强制实测断点：长距离路线和重新捕获已完成，
                    // 在用户确认实际落点以前绝不继续靠近或踢球。
                    approachStatus = "STAGING TEST COMPLETE - STOP";
                }

                approachStatus += detectedThisFrame ? " live" : " stale";

                cv::rectangle(
                    image,
                    displayRect,
                    cv::Scalar(255, 0, 0),
                    3
                );
                if (nearBallCircleRecovered) {
                    cv::circle(
                        image,
                        measuredBallCenter,
                        measuredBallSize / 2,
                        cv::Scalar(0, 255, 0),
                        2
                    );
                }
                cv::circle(
                    image,
                    measuredBallCenter,
                    5,
                    cv::Scalar(0, 0, 255),
                    -1
                );
                cv::putText(
                    image,
                    nearBallCircleRecovered
                        ? "ball full-circle"
                        : closeBallGeometryRequired
                            ? "ball full-circle stale"
                            : "ball candidate",
                    cv::Point(displayRect.x, std::max(20, displayRect.y - 8)),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(255, 0, 0),
                    2
                );
                cv::putText(
                    image,
                    approachStatus + " size=" + std::to_string(measuredBallSize),
                    cv::Point(20, 35),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.65,
                    cv::Scalar(255, 0, 0),
                    2
                );
            } else if (foundBallCandidate) {
                cv::putText(
                    image,
                    "ball: confirming " + std::to_string(stableBallFrames) + "/3",
                    cv::Point(20, 35),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    cv::Scalar(255, 0, 0),
                    2
                );
            } else {
                cv::putText(
                    image,
                    closeBallGeometryRequired
                        ? std::string(
                            approachPhase == ApproachPhase::BALL_REACQUIRE
                                ? "T4 ball miss "
                                : approachPhase ==
                                    ApproachPhase::COARSE_BALL_REACQUIRE
                                    ? "T5 ball miss "
                                    : approachPhase ==
                                        ApproachPhase::FORWARD_BALL_REACQUIRE
                                        ? "T6 ball miss "
                                        : "shot-ready ball miss "
                        ) +
                            std::to_string(reacquireMissFrames) + "/" +
                            std::to_string(reacquireMissFrameLimit) +
                            " circles=" +
                            std::to_string(nearCircleProposalCount) + "/" +
                            std::to_string(nearCircleAcceptedCount) +
                            " yaw=" + std::to_string(headTargetYaw) +
                            " pitch=" + std::to_string(headTargetPitch)
                        : "ball: none",
                    cv::Point(20, 35),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    cv::Scalar(255, 0, 0),
                    2
                );
            }
        }

        if ((gameData.state == gameData.STATE_PLAY ||
            (approachPhase == ApproachPhase::KICK_STOP && kickIssued)) &&
            newCameraFrame &&
            !image.empty() &&
            approachPhase != ApproachPhase::OBSERVE) {
            std::string motionStatus = "body: waiting initial turn";
            if (!robotUpright) {
                motionStatus = "FALL RECOVERY - route timer paused";
            } else if (approachPhase == ApproachPhase::BODY_TURN) {
                const int turnFrame =
                    turnCommandFrames - approachPhaseFrames + 1;
                motionStatus = "body: turning, vision suspended " +
                    std::to_string(std::max(1, turnFrame)) + "/" +
                    std::to_string(turnCommandFrames) + " cmd=" +
                    std::to_string(activeTurnCommand) + " pulse=" +
                    std::to_string(turnPulseNumber);
            } else if (approachPhase == ApproachPhase::TURN_SETTLE) {
                motionStatus = "body: settling, vision suspended yaw=" +
                    std::to_string(static_cast<double>(imuData.yaw));
                if (imuTurnDirectionKnown) {
                    motionStatus += " target=" + std::to_string(targetImuYaw);
                }
            } else if (approachPhase == ApproachPhase::LONG_WALK) {
                motionStatus = "route: walking " +
                    std::to_string(longWalkProcessedFrames) + "/" +
                    std::to_string(longWalkCommandFrames) +
                    " turn=" + std::to_string(walkTurnCommand) +
                    " err=" + std::to_string(filteredWalkHeadingError);
            } else if (approachPhase == ApproachPhase::WALK_SETTLE) {
                const int settleFrame =
                    walkSettleFrames - approachPhaseFrames + 1;
                motionStatus = "route: walk settling " +
                    std::to_string(std::max(1, settleFrame)) + "/" +
                    std::to_string(walkSettleFrames);
            } else if (approachPhase == ApproachPhase::BALL_FACING_TURN) {
                const int turnFrame =
                    turnCommandFrames - approachPhaseFrames + 1;
                motionStatus = "body: aligning shot line " +
                    (ballFacingCorrectionPulses > 0
                        ? "correct " +
                            std::to_string(ballFacingCorrectionPulses) + "/" +
                            std::to_string(maxBallFacingCorrectionPulses) + " "
                        : "") +
                    std::to_string(std::max(1, turnFrame)) + "/" +
                    std::to_string(turnCommandFrames) + " cmd=" +
                    std::to_string(activeTurnCommand);
            } else if (approachPhase == ApproachPhase::BALL_FACING_SETTLE) {
                motionStatus = "body: shot turn settling yaw=" +
                    std::to_string(static_cast<double>(imuData.yaw));
            } else if (approachPhase == ApproachPhase::GOAL_VIEW_TEST) {
                const bool headLevel =
                    std::abs(static_cast<double>(headAngle.yaw)) <= 3.0 &&
                    std::abs(static_cast<double>(headAngle.pitch)) <= 3.0;
                if (ballFacingAlignmentSafetyStop) {
                    motionStatus = "STOP: shot yaw error=" +
                        std::to_string(ballFacingPhysicalError);
                } else if (lockedShotSide != 0) {
                    motionStatus = "T2 SIDE LOCKED IMAGE-" +
                        std::string(lockedShotSide < 0 ? "LEFT" : "RIGHT") +
                        " - STOP";
                } else if (hasFrozenShotGoal) {
                    motionStatus = "T2 frozen goal prefer=" +
                        std::string(
                            pendingShotSide < 0 ? "LEFT" :
                            pendingShotSide > 0 ? "RIGHT" : "WAIT"
                        ) + " " + std::to_string(shotSideStableFrames) +
                        "/" + std::to_string(shotSideConfirmFrames) +
                        " wait=" + std::to_string(shotDecisionWaitFrames) +
                        "/" + std::to_string(shotDecisionMaxWaitFrames);
                } else if (goalStableFrames >= goalConfirmFrames &&
                    keeperStableFrames >= keeperConfirmFrames) {
                    motionStatus = "T2 tracking keeper prefer=" +
                        std::string(
                            pendingShotSide < 0 ? "LEFT" :
                            pendingShotSide > 0 ? "RIGHT" : "NONE"
                        ) + " " + std::to_string(shotSideStableFrames) +
                        "/" + std::to_string(shotSideConfirmFrames);
                } else if (headLevel && goalViewStableFrames >= 3 &&
                    goalViewLocationSamples >= 10) {
                    motionStatus = "T1 detect goal=" +
                        std::to_string(goalStableFrames) + "/" +
                        std::to_string(goalConfirmFrames) + " keeper=" +
                        std::to_string(keeperStableFrames) + "/" +
                        std::to_string(keeperConfirmFrames);
                } else {
                    motionStatus = "goal: leveling head pitch=0 " +
                        std::to_string(goalViewStableFrames) + "/3 loc=" +
                        std::to_string(goalViewLocationSamples) + "/10";
                }
            } else if (approachPhase == ApproachPhase::SHOT_TARGET_TURN) {
                const int turnFrame =
                    turnCommandFrames - approachPhaseFrames + 1;
                motionStatus = "T3 target turn " +
                    std::to_string(std::max(1, turnFrame)) + "/" +
                    std::to_string(turnCommandFrames) + " cmd=" +
                    std::to_string(activeTurnCommand);
            } else if (approachPhase ==
                ApproachPhase::SHOT_TARGET_SETTLE) {
                motionStatus = "T3 settling yaw=" +
                    std::to_string(static_cast<double>(imuData.yaw)) +
                    " target=" + std::to_string(shotTargetImuYaw);
            } else if (approachPhase == ApproachPhase::SHOT_TARGET_STOP) {
                motionStatus = shotTargetAlignmentSafetyStop
                    ? "T3 STOP: target yaw alignment failed"
                    : "T3 SHOT TARGET ALIGNED - STOP";
            } else if (approachPhase == ApproachPhase::BALL_REACQUIRE) {
                motionStatus = "T4 static ball reacquire yaw=" +
                    std::to_string(headTargetYaw) + " pitch=" +
                    std::to_string(headTargetPitch);
            } else if (approachPhase ==
                ApproachPhase::BALL_REACQUIRE_STOP) {
                motionStatus = ballReacquireSucceeded
                    ? "T4 GEOMETRY CONFIRMED - STOP"
                    : "T4 STOP: ball not found";
            } else if (approachPhase ==
                ApproachPhase::COARSE_LATERAL_MOVE) {
                motionStatus = "T5 one lateral pulse cmd=" +
                    std::to_string(coarseLateralCommand) + " frame=" +
                    std::to_string(
                        coarseMotionCommandFrames -
                            approachPhaseFrames + 1
                    ) + "/" +
                    std::to_string(coarseMotionCommandFrames);
            } else if (approachPhase ==
                ApproachPhase::COARSE_LATERAL_SETTLE) {
                motionStatus = "T5 lateral settling - vision frozen";
            } else if (approachPhase ==
                ApproachPhase::COARSE_YAW_RECOVER) {
                motionStatus = "T5 restoring shot yaw cmd=" +
                    std::to_string(activeTurnCommand);
            } else if (approachPhase ==
                ApproachPhase::COARSE_YAW_SETTLE) {
                motionStatus = "T5 shot yaw settling";
            } else if (approachPhase ==
                ApproachPhase::COARSE_BALL_REACQUIRE) {
                motionStatus = "T5 ball " +
                    std::to_string(reacquireMatchFrames) + "/" +
                    std::to_string(reacquireConfirmFrames);
            } else if (approachPhase ==
                ApproachPhase::COARSE_LATERAL_STOP) {
                motionStatus = coarseSafetyStop
                    ? "T5 STOP: lateral/yaw result needs review"
                    : coarseAlignmentReached
                        ? "T5 RIGHT-FOOT WINDOW CONFIRMED - STOP"
                        : "T5 BOUNDED ALIGNMENT INCOMPLETE - STOP";
            } else if (approachPhase ==
                ApproachPhase::FORWARD_CALIBRATION_MOVE) {
                motionStatus = "T6 one forward pulse step=" +
                    std::to_string(forwardCalibrationStep) + " frame=" +
                    std::to_string(
                        forwardCalibrationCommandFrames -
                            approachPhaseFrames + 1
                    ) + "/" +
                    std::to_string(forwardCalibrationCommandFrames);
            } else if (approachPhase ==
                ApproachPhase::FORWARD_CALIBRATION_SETTLE) {
                motionStatus = "T6 forward settling - vision frozen";
            } else if (approachPhase ==
                ApproachPhase::FORWARD_YAW_RECOVER) {
                motionStatus = "T6 restoring shot yaw cmd=" +
                    std::to_string(activeTurnCommand);
            } else if (approachPhase ==
                ApproachPhase::FORWARD_YAW_SETTLE) {
                motionStatus = "T6 shot yaw settling";
            } else if (approachPhase ==
                ApproachPhase::FORWARD_BALL_REACQUIRE) {
                motionStatus = "T6 static ball reacquire " +
                    std::to_string(reacquireMatchFrames) + "/" +
                    std::to_string(reacquireConfirmFrames);
            } else if (approachPhase ==
                ApproachPhase::FORWARD_CALIBRATION_STOP) {
                motionStatus = "T6 STOP: calibration result needs review";
            } else if (approachPhase == ApproachPhase::FINE_MOVE) {
                motionStatus = "shot-ready move step=" +
                    std::to_string(fineStepCommand) + " lateral=" +
                    std::to_string(fineLateralCommand) + " frame=" +
                    std::to_string(
                        fineMotionCommandFrames - approachPhaseFrames + 1
                    ) + "/" + std::to_string(fineMotionCommandFrames);
            } else if (approachPhase == ApproachPhase::FINE_SETTLE) {
                motionStatus = "shot-ready settling - vision frozen";
            } else if (approachPhase ==
                ApproachPhase::FINE_YAW_RECOVER) {
                motionStatus = "shot-ready restoring compensated yaw cmd=" +
                    std::to_string(activeTurnCommand);
            } else if (approachPhase ==
                ApproachPhase::FINE_YAW_SETTLE) {
                motionStatus = "shot-ready yaw settling";
            } else if (approachPhase ==
                ApproachPhase::FINE_BALL_REACQUIRE) {
                motionStatus = "shot-ready static ball reacquire " +
                    std::to_string(reacquireMatchFrames) + "/" +
                    std::to_string(reacquireConfirmFrames);
            } else if (approachPhase == ApproachPhase::KICK_REQUEST) {
                motionStatus = "RIGHT KICK REQUESTED - latched frame=" +
                    std::to_string(
                        kickRequestFrames - approachPhaseFrames + 1
                    ) + "/" + std::to_string(kickRequestFrames);
            } else if (approachPhase == ApproachPhase::KICK_WAIT) {
                motionStatus = "RIGHT KICK IN PROGRESS - zero walk";
            } else if (approachPhase == ApproachPhase::KICK_STOP) {
                motionStatus = fineSafetyStop
                    ? std::string("SHOT STOP: ") + fineStopReason
                    : kickWaitCompleted
                        ? "KICK ACTION WINDOW ENDED - RESULT UNVERIFIED"
                        : "SHOT STOP: kick not completed";
            }

            cv::putText(
                image,
                motionStatus,
                cv::Point(20, 35),
                cv::FONT_HERSHEY_SIMPLEX,
                0.65,
                cv::Scalar(255, 0, 0),
                2
            );

            if (approachPhase == ApproachPhase::BALL_REACQUIRE_STOP) {
                if (ballReacquireSucceeded &&
                    trackedBallRect.width > 0 &&
                    trackedBallRect.height > 0) {
                    const cv::Rect safeBallRect = trackedBallRect &
                        cv::Rect(0, 0, image.cols, image.rows);
                    if (safeBallRect.width > 0 && safeBallRect.height > 0) {
                        cv::rectangle(
                            image,
                            safeBallRect,
                            cv::Scalar(255, 0, 0),
                            3
                        );
                        cv::circle(
                            image,
                            trackedBallCenter,
                            std::min(
                                safeBallRect.width,
                                safeBallRect.height
                            ) / 2,
                            cv::Scalar(0, 255, 0),
                            2
                        );
                        cv::circle(
                            image,
                            trackedBallCenter,
                            6,
                            cv::Scalar(0, 0, 255),
                            -1
                        );
                    }
                }
                cv::putText(
                    image,
                    "T4 head yaw=" +
                        std::to_string(ballReacquireHeadYaw) +
                        " pitch=" +
                        std::to_string(ballReacquireHeadPitch) +
                        " safety=" +
                        std::to_string(ballReacquireSafetyStop ? 1 : 0),
                    cv::Point(20, 245),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.55,
                    cv::Scalar(255, 0, 0),
                    2
                );
                cv::putText(
                    image,
                    "T4 ballCenter=(" +
                        std::to_string(trackedBallCenter.x) + "," +
                        std::to_string(trackedBallCenter.y) + ") radius=" +
                        std::to_string(nearBallConfirmedRadius) +
                        " samples=" +
                        std::to_string(nearBallGeometrySamples),
                    cv::Point(20, 275),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.50,
                    cv::Scalar(255, 0, 0),
                    2
                );
            }

            if (approachPhase == ApproachPhase::COARSE_LATERAL_STOP) {
                if (coarseAfterBallX >= 0 &&
                    trackedBallRect.width > 0 &&
                    trackedBallRect.height > 0) {
                    const cv::Rect safeBallRect = trackedBallRect &
                        cv::Rect(0, 0, image.cols, image.rows);
                    cv::rectangle(
                        image,
                        safeBallRect,
                        cv::Scalar(255, 0, 0),
                        3
                    );
                    cv::circle(
                        image,
                        trackedBallCenter,
                        nearBallConfirmedRadius,
                        cv::Scalar(0, 255, 0),
                        2
                    );
                    cv::circle(
                        image,
                        trackedBallCenter,
                        6,
                        cv::Scalar(0, 0, 255),
                        -1
                    );
                }
                cv::putText(
                    image,
                    "T5 x=" + std::to_string(coarseInitialBallX) +
                        "->" + std::to_string(coarseAfterBallX) +
                        " shift=" + std::to_string(coarsePixelShift) +
                        " pulses=" +
                        std::to_string(coarseLateralPulses) + "/" +
                        std::to_string(maxCoarseLateralPulses),
                    cv::Point(20, 245),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.50,
                    cv::Scalar(255, 0, 0),
                    2
                );
                cv::putText(
                    image,
                    "T5 direction=" + std::string(
                        coarseDirectionMatched ? "MATCHED" : "WRONG/UNKNOWN"
                    ) + " headingErr=" +
                        std::to_string(coarseHeadingError) +
                        " yawFix=" +
                        std::to_string(coarseYawCorrectionPulses),
                    cv::Point(20, 275),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.50,
                    cv::Scalar(255, 0, 0),
                    2
                );
                cv::putText(
                    image,
                    "T5 rightFoot targetX=" +
                        std::to_string(preferredFootTargetX) + " +/-" +
                        std::to_string(preferredFootTolerancePixels) +
                        " radius=" +
                        std::to_string(nearBallConfirmedRadius),
                    cv::Point(20, 305),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.50,
                    cv::Scalar(255, 0, 0),
                    2
                );
            }

            if (approachPhase ==
                ApproachPhase::FORWARD_CALIBRATION_STOP) {
                if (forwardAfterBallX >= 0 &&
                    trackedBallRect.width > 0 &&
                    trackedBallRect.height > 0) {
                    const cv::Rect safeBallRect = trackedBallRect &
                        cv::Rect(0, 0, image.cols, image.rows);
                    cv::rectangle(
                        image,
                        safeBallRect,
                        cv::Scalar(255, 0, 0),
                        3
                    );
                    cv::circle(
                        image,
                        trackedBallCenter,
                        nearBallConfirmedRadius,
                        cv::Scalar(0, 255, 0),
                        2
                    );
                    cv::circle(
                        image,
                        trackedBallCenter,
                        6,
                        cv::Scalar(0, 0, 255),
                        -1
                    );
                }
                cv::putText(
                    image,
                    "T6 y=" + std::to_string(forwardBeforeBallY) +
                        "->" + std::to_string(forwardAfterBallY) +
                        " dy=" + std::to_string(forwardDeltaBallY) +
                        " step=" +
                        std::to_string(forwardCalibrationStep),
                    cv::Point(20, 245),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.50,
                    cv::Scalar(255, 0, 0),
                    2
                );
                cv::putText(
                    image,
                    "T6 radius=" +
                        std::to_string(forwardBeforeRadius) + "->" +
                        std::to_string(forwardAfterRadius) + " dr=" +
                        std::to_string(forwardDeltaRadius) + " x=" +
                        std::to_string(forwardBeforeBallX) + "->" +
                        std::to_string(forwardAfterBallX),
                    cv::Point(20, 275),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.50,
                    cv::Scalar(255, 0, 0),
                    2
                );
                cv::putText(
                    image,
                    "T6 headingErr=" +
                        std::to_string(forwardBeforeHeadingError) + "->" +
                        std::to_string(forwardAfterHeadingError) +
                        " yawFix=" +
                        std::to_string(forwardYawCorrectionPulses) +
                        " reacquire=" +
                        std::to_string(forwardReacquireSucceeded ? 1 : 0),
                    cv::Point(20, 305),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.50,
                    cv::Scalar(255, 0, 0),
                    2
                );
            }

            if (approachPhase == ApproachPhase::KICK_STOP) {
                cv::putText(
                    image,
                    "FINAL ball=(" + std::to_string(finalBallX) + "," +
                        std::to_string(finalBallY) + ") radius=" +
                        std::to_string(finalBallRadius) + " targetX=" +
                        std::to_string(preferredFootTargetX) + " +/-" +
                        std::to_string(preferredFootTolerancePixels),
                    cv::Point(20, 245),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.47,
                    cv::Scalar(255, 0, 0),
                    2
                );
                cv::putText(
                    image,
                    "FINAL headingErr=" +
                        std::to_string(finalHeadingError) + " comp=" +
                        std::to_string(
                            rightFootKickAimCompensationDegrees
                        ) + " forward=" +
                        std::to_string(fineForwardPulses) + " lateral=" +
                        std::to_string(fineLateralPulses),
                    cv::Point(20, 275),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.47,
                    cv::Scalar(255, 0, 0),
                    2
                );
                cv::putText(
                    image,
                    "KICK issued=" + std::to_string(kickIssued ? 1 : 0) +
                        " waited=" +
                        std::to_string(kickWaitCompleted ? 1 : 0) +
                        " fall=" +
                        std::to_string(kickFallObserved ? 1 : 0) +
                        " remain=" +
                        std::to_string(kickStartRemainTime) + "->" +
                        std::to_string(kickStopRemainTime),
                    cv::Point(20, 305),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.47,
                    cv::Scalar(255, 0, 0),
                    2
                );
                cv::putText(
                    image,
                    "SCORE red=" + std::to_string(gameData.red_score) +
                        " blue=" + std::to_string(gameData.blue_score) +
                        " state=" + std::to_string(gameData.state),
                    cv::Point(20, 335),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.47,
                    cv::Scalar(255, 0, 0),
                    2
                );
            }
        }

        // FORWARD_CALIBRATION_STOP 和 KICK_STOP 是异常/动作后断点，
        // 两者均持续发布零行走；正常近球链不会被它们提前截断。
        // 开头清零，而且非零发布分支不包含该状态，因此停在这里时持续
        // 停止，只有下一次 INIT 能重置整条路线。

        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            !image.empty() && !robotUpright &&
            approachPhase == ApproachPhase::OBSERVE) {
            cv::putText(
                image,
                "FALL RECOVERY - vision paused",
                cv::Point(20, 35),
                cv::FONT_HERSHEY_SIMPLEX,
                0.75,
                cv::Scalar(255, 0, 0),
                2
            );
        }

        // 初始理论角一次完成，停稳确认 IMU 符号后立即进入连续长距离
        // 行走；初始欠转量由行走中的航向闭环吸收，不再停车视觉微调。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright &&
            (approachPhase == ApproachPhase::BODY_TURN ||
             approachPhase == ApproachPhase::TURN_SETTLE)) {
            if (approachPhaseFrames > 0) {
                approachPhaseFrames = std::max(
                    0,
                    approachPhaseFrames - motionFrameAdvance
                );
            }
            if (approachPhaseFrames == 0) {
                if (approachPhase == ApproachPhase::BODY_TURN) {
                    approachPhase = ApproachPhase::TURN_SETTLE;
                    approachPhaseFrames = turnSettleFrames;
                    liveBallFrames = 0;
                    btask.step = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else {
                    const double currentYaw =
                        static_cast<double>(imuData.yaw);
                    const double observedPulseYaw = normalizeAngle(
                        currentYaw - turnPulseStartYaw
                    );

                    // 初始大角度只执行一次。停稳后由实际 yaw 变化自动
                    // 确认 IMU 符号，并建立整个长距离阶段的绝对目标航向。
                    if (std::abs(observedPulseYaw) >= 1.0) {
                        imuYawDirectionForPositiveTurn =
                            observedPulseYaw * activeTurnCommand >= 0.0
                                ? 1.0
                                : -1.0;
                        imuTurnDirectionKnown = true;
                        targetImuYaw = normalizeAngle(
                            initYaw +
                            imuYawDirectionForPositiveTurn *
                                nominalInitialLeftTurn
                        );
                        const double imuYawError = normalizeAngle(
                            targetImuYaw - currentYaw
                        );
                        latestPhysicalTurnError =
                            imuYawError * imuYawDirectionForPositiveTurn;
                        filteredWalkHeadingError = latestPhysicalTurnError;
                        walkTurnCommand = 0.0;
                        initialGeometryTurnComplete = true;
                        routeTravelStarted = true;
                        longWalkProcessedFrames = 0;
                        longWalkStartRemainTime = gameData.remain_time;
                        approachPhase = ApproachPhase::LONG_WALK;
                        approachPhaseFrames = longWalkCommandFrames;
                        enteredLongWalkThisFrame = true;
                        headTargetYaw = 0.0;
                        headTargetPitch = 20.0;
                        htask.yaw = headTargetYaw;
                        htask.pitch = headTargetPitch;
                    } else {
                        // 无法确认 IMU 转向符号时禁止开始三米盲走。
                        initialTurnSafetyStop = true;
                        approachPhase = ApproachPhase::OBSERVE;
                    }
                }
            }
        }

        // 连续长距离阶段不再中途停车。相机每 100ms 仿真时间发布一帧，
        // 因而 260 个新画面对应约 26 个仿真秒。当前步长 0.05 m
        // 已是 walk.conf 允许的上限，不能在 player.cpp 内再通过增大步长加速。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright &&
            !enteredLongWalkThisFrame &&
            (approachPhase == ApproachPhase::LONG_WALK ||
             approachPhase == ApproachPhase::WALK_SETTLE)) {
            if (approachPhaseFrames > 0) {
                approachPhaseFrames = std::max(
                    0,
                    approachPhaseFrames - motionFrameAdvance
                );
            }
            if (approachPhase == ApproachPhase::LONG_WALK) {
                longWalkProcessedFrames = std::min(
                    longWalkCommandFrames,
                    longWalkProcessedFrames + motionFrameAdvance
                );
            }
            if (approachPhaseFrames == 0) {
                if (approachPhase == ApproachPhase::LONG_WALK) {
                    approachPhase = ApproachPhase::WALK_SETTLE;
                    approachPhaseFrames = walkSettleFrames;
                    btask.step = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else {
                    finalWalkHeadingError = normalizeAngle(
                        targetImuYaw - static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    routeTravelComplete = true;
                    routeCompleteRemainTime = gameData.remain_time;
                    // 长走终点是球正后方约 0.5 m 的射门预备位。
                    // 再按剩余几何角转到红方进攻方向，使身体、球和
                    // 球门中心共线。转动期间继续暂停视觉。
                    turnPulseStartYaw = static_cast<double>(imuData.yaw);
                    ballFacingTargetImuYaw = normalizeAngle(
                        initYaw +
                        imuYawDirectionForPositiveTurn *
                            nominalAttackTurnFromStart
                    );
                    const double remainingAttackTurn = normalizeAngle(
                        ballFacingTargetImuYaw - turnPulseStartYaw
                    ) * imuYawDirectionForPositiveTurn;
                    activeTurnCommand =
                        remainingAttackTurn /
                        static_cast<double>(turnCycleCount);
                    ballFacingCorrectionPulses = 0;
                    ballFacingAlignmentSafetyStop = false;
                    approachPhase = ApproachPhase::BALL_FACING_TURN;
                    approachPhaseFrames = turnCommandFrames;
                    enteredBallFacingTurnThisFrame = true;
                    hasBallTrack = false;
                    lostBallFrames = 0;
                    settledLostBallFrames = 0;
                    stableBallFrames = 0;
                    liveBallFrames = 0;
                    localReacquirePoseIndex = 0;
                    headTargetYaw = 0.0;
                    // 转身与抬头并行，减少转身完成后额外等待平视的时间。
                    headTargetPitch = 0.0;
                    htask.yaw = headTargetYaw;
                    htask.pitch = headTargetPitch;
                    reacquireAfterTurn = false;
                    reacquireMatchFrames = 0;
                    reacquireMissFrames = 0;
                }
            }
        }

        // 第二个大转向只执行一次。本轮先不低头找球，
        // 而是将头部设为 yaw=0、pitch=0 平视球门，用于同时
        // 验证落点、射门航向和球门/守门员在画面中的位置。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright &&
            !enteredBallFacingTurnThisFrame &&
            (approachPhase == ApproachPhase::BALL_FACING_TURN ||
             approachPhase == ApproachPhase::BALL_FACING_SETTLE)) {
            if (approachPhaseFrames > 0) {
                approachPhaseFrames = std::max(
                    0,
                    approachPhaseFrames - motionFrameAdvance
                );
            }
            if (approachPhaseFrames == 0) {
                if (approachPhase == ApproachPhase::BALL_FACING_TURN) {
                    approachPhase = ApproachPhase::BALL_FACING_SETTLE;
                    approachPhaseFrames = turnSettleFrames;
                    btask.step = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else {
                    ballFacingPhysicalError = normalizeAngle(
                        ballFacingTargetImuYaw -
                        static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    if (std::abs(ballFacingPhysicalError) >
                            ballFacingYawAcceptDegrees &&
                        ballFacingCorrectionPulses <
                            maxBallFacingCorrectionPulses) {
                        // 每个补转脉冲都被限制在较小范围，并在下一次
                        // 停稳后重新读取 IMU；绝不根据旧误差连续叠加转身。
                        activeTurnCommand = std::max(
                            -maxBallFacingCorrectionPerCycle,
                            std::min(
                                maxBallFacingCorrectionPerCycle,
                                ballFacingPhysicalError /
                                    static_cast<double>(turnCycleCount)
                            )
                        );
                        turnPulseStartYaw =
                            static_cast<double>(imuData.yaw);
                        ++ballFacingCorrectionPulses;
                        approachPhase = ApproachPhase::BALL_FACING_TURN;
                        approachPhaseFrames = turnCommandFrames;
                        btask.step = 0.0;
                        btask.turn = 0.0;
                        btask.count = 0;
                    }
                    if (approachPhase == ApproachPhase::BALL_FACING_SETTLE) {
                        ballFacingTurnComplete = true;
                        ballFacingAlignmentSafetyStop =
                            std::abs(ballFacingPhysicalError) >
                                ballFacingYawAcceptDegrees;
                        approachPhase = ApproachPhase::GOAL_VIEW_TEST;
                        hasBallTrack = false;
                        lostBallFrames = 0;
                        settledLostBallFrames = 0;
                        stableBallFrames = 0;
                        liveBallFrames = 0;
                        localReacquirePoseIndex = 0;
                        headTargetYaw = 0.0;
                        headTargetPitch = 0.0;
                        htask.yaw = headTargetYaw;
                        htask.pitch = headTargetPitch;
                        reacquireAfterTurn = false;
                        reacquireMatchFrames = 0;
                        reacquireMissFrames = 0;
                        goalViewStableFrames = 0;
                        goalViewLocationSamples = 0;
                        goalViewLocationXSum = 0.0;
                        goalViewLocationZSum = 0.0;
                        hasGoalTrack = false;
                        trackedGoalRect = cv::Rect();
                        goalStableFrames = 0;
                        goalMissFrames = 0;
                        hasKeeperTrack = false;
                        trackedKeeperRect = cv::Rect();
                        keeperStableFrames = 0;
                        keeperMissFrames = 0;
                        t1KeeperAnchorVisible = false;
                        t1KeeperAnchorRect = cv::Rect();
                        t1VerticalLineCandidates = 0;
                        t1GoalDetectionMethod = 0;
                        keeperMotionReady = false;
                        filteredKeeperU = 0.5;
                        leftShotClearance = 0.0;
                        rightShotClearance = 0.0;
                        keeperMotionSamples = 0;
                        pendingShotSide = 0;
                        shotSideStableFrames = 0;
                        lockedShotSide = 0;
                        hasFrozenShotGoal = false;
                        frozenShotGoalRect = cv::Rect();
                        shotDecisionWaitFrames = 0;
                        shotTargetVisualAngleDegrees = 0.0;
                        shotTargetImuYaw =
                            static_cast<double>(imuData.yaw);
                        shotTargetPhysicalError = 0.0;
                        shotTargetCorrectionPulses = 0;
                        shotTargetAlignmentSafetyStop = false;
                    }
                }
            }
        }

        // 平视头部连续到位 5 帧后仍保持全零 BodyTask；同时对允许使用的
        // 约 1 m 精度定位做 10 帧均值，只作为本轮路线校准诊断，不把它
        // 当作足球真值或最终近距离控制依据。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright &&
            approachPhase == ApproachPhase::GOAL_VIEW_TEST) {
            const bool headLevel =
                std::abs(static_cast<double>(headAngle.yaw)) <= 3.0 &&
                std::abs(static_cast<double>(headAngle.pitch)) <= 3.0;
            if (headLevel) {
                goalViewStableFrames = std::min(goalViewStableFrames + 1, 5);
            } else {
                goalViewStableFrames = 0;
            }
            if (goalViewLocationSamples < 10) {
                goalViewLocationXSum += static_cast<double>(location.x);
                goalViewLocationZSum += static_cast<double>(location.z);
                ++goalViewLocationSamples;
            }
        }

        // T1/T2：在机器人、头部均静止的平视图中检测球门和蓝方机器人，
        // 冻结可靠门框并一次性选择射门点；完成前不发布身体动作。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright && !image.empty() &&
            approachPhase == ApproachPhase::GOAL_VIEW_TEST &&
            !ballFacingAlignmentSafetyStop &&
            goalViewStableFrames >= 3) {
            cv::Mat goalHsv;
            cv::cvtColor(image, goalHsv, cv::COLOR_RGB2HSV);

            cv::Mat goalWhiteMask;
            cv::inRange(
                goalHsv,
                cv::Scalar(0, 0, 150),
                cv::Scalar(180, 85, 255),
                goalWhiteMask
            );
            const cv::Mat goalKernel = cv::getStructuringElement(
                cv::MORPH_RECT,
                cv::Size(5, 5)
            );
            cv::morphologyEx(
                goalWhiteMask,
                goalWhiteMask,
                cv::MORPH_CLOSE,
                goalKernel
            );

            cv::Mat goalGreenMask;
            cv::inRange(
                goalHsv,
                cv::Scalar(25, 50, 30),
                cv::Scalar(95, 255, 255),
                goalGreenMask
            );

            bool foundGoalThisFrame = false;
            cv::Rect bestGoalRect;
            double bestGoalScore = -1.0;
            const cv::Rect fullImageRect(0, 0, image.cols, image.rows);
            t1KeeperAnchorVisible = false;
            t1KeeperAnchorRect = cv::Rect();
            t1VerticalLineCandidates = 0;
            t1GoalDetectionMethod = 0;

            // 蓝方胸甲比天空具有更高的饱和度。先在合理高度寻找小型
            // 蓝色连通域，把门将作为球门几何的锚点；天空的大蓝块和
            // 靠近画面顶端的云层因此不会进入候选。
            cv::Mat keeperColorMask;
            cv::inRange(
                goalHsv,
                cv::Scalar(95, 140, 30),
                cv::Scalar(140, 255, 255),
                keeperColorMask
            );
            const cv::Mat keeperKernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(3, 3)
            );
            cv::morphologyEx(
                keeperColorMask,
                keeperColorMask,
                cv::MORPH_OPEN,
                keeperKernel
            );
            cv::morphologyEx(
                keeperColorMask,
                keeperColorMask,
                cv::MORPH_CLOSE,
                keeperKernel
            );
            std::vector<std::vector<cv::Point>> anchorContours;
            cv::Mat anchorContourMask = keeperColorMask.clone();
            cv::findContours(
                anchorContourMask,
                anchorContours,
                cv::RETR_EXTERNAL,
                cv::CHAIN_APPROX_SIMPLE
            );
            double bestAnchorArea = 0.0;
            for (const auto &anchorContour : anchorContours) {
                const double area = cv::contourArea(anchorContour);
                const cv::Rect rect =
                    cv::boundingRect(anchorContour) & fullImageRect;
                const cv::Point center(
                    rect.x + rect.width / 2,
                    rect.y + rect.height / 2
                );
                if (area < 8.0 ||
                    area > image.cols * image.rows * 0.02 ||
                    center.y < image.rows * 0.20 ||
                    center.y > image.rows * 0.68 ||
                    rect.width > image.cols * 0.12 ||
                    rect.height > image.rows * 0.25) {
                    continue;
                }
                if (area > bestAnchorArea) {
                    bestAnchorArea = area;
                    t1KeeperAnchorRect = rect;
                    t1KeeperAnchorVisible = true;
                }
            }

            // 优先方案：以门将为锚点，在灰度边缘中寻找包围门将的
            // 左右长立柱，并检查两柱顶部之间确有横梁边缘。
            if (t1KeeperAnchorVisible) {
                cv::Mat goalGray;
                cv::Mat goalEdges;
                cv::cvtColor(image, goalGray, cv::COLOR_RGB2GRAY);
                cv::Canny(goalGray, goalEdges, 80.0, 200.0);
                std::vector<cv::Vec4i> edgeLines;
                cv::HoughLinesP(
                    goalEdges,
                    edgeLines,
                    1.0,
                    CV_PI / 180.0,
                    20,
                    image.rows * 0.07,
                    12.0
                );
                struct AnchoredGoalLine {
                    int x;
                    int top;
                    int bottom;
                };
                std::vector<AnchoredGoalLine> anchoredVerticalLines;
                for (const auto &line : edgeLines) {
                    const int dx = line[2] - line[0];
                    const int dy = line[3] - line[1];
                    const int top = std::min(line[1], line[3]);
                    const int bottom = std::max(line[1], line[3]);
                    const int middle = (top + bottom) / 2;
                    if (std::abs(dy) < image.rows * 0.07 ||
                        std::abs(dx) > std::abs(dy) * 0.30 ||
                        middle < image.rows * 0.10 ||
                        middle > image.rows * 0.62) {
                        continue;
                    }
                    anchoredVerticalLines.push_back({
                        (line[0] + line[2]) / 2,
                        top,
                        bottom
                    });
                }
                t1VerticalLineCandidates = static_cast<int>(
                    anchoredVerticalLines.size()
                );
                const cv::Point anchorCenter(
                    t1KeeperAnchorRect.x + t1KeeperAnchorRect.width / 2,
                    t1KeeperAnchorRect.y + t1KeeperAnchorRect.height / 2
                );
                for (std::size_t i = 0;
                     i < anchoredVerticalLines.size(); ++i) {
                    for (std::size_t j = i + 1;
                         j < anchoredVerticalLines.size(); ++j) {
                        const AnchoredGoalLine &first =
                            anchoredVerticalLines[i];
                        const AnchoredGoalLine &second =
                            anchoredVerticalLines[j];
                        const AnchoredGoalLine &left =
                            first.x <= second.x ? first : second;
                        const AnchoredGoalLine &right =
                            first.x <= second.x ? second : first;
                        const int separation = right.x - left.x;
                        if (separation < image.cols * 0.18 ||
                            separation > image.cols * 0.70 ||
                            left.x >= anchorCenter.x - image.cols * 0.03 ||
                            right.x <= anchorCenter.x + image.cols * 0.03 ||
                            std::abs(left.top - right.top) >
                                image.rows * 0.12 ||
                            left.top > anchorCenter.y - image.rows * 0.05 ||
                            right.top > anchorCenter.y - image.rows * 0.05 ||
                            left.bottom <
                                anchorCenter.y - image.rows * 0.04 ||
                            right.bottom <
                                anchorCenter.y - image.rows * 0.04) {
                            continue;
                        }
                        const int top = std::min(left.top, right.top);
                        const int bottom = std::max(
                            left.bottom,
                            right.bottom
                        );
                        const int height = bottom - top + 1;
                        if (separation < height * 1.4 ||
                            separation > height * 4.2) {
                            continue;
                        }

                        const int topBandY = std::max(
                            0,
                            top - image.rows / 50
                        );
                        const int topBandHeight = std::min(
                            std::max(8, image.rows / 15),
                            image.rows - topBandY
                        );
                        const cv::Rect topBand(
                            left.x,
                            topBandY,
                            separation + 1,
                            topBandHeight
                        );
                        const double topEdgeRatio = static_cast<double>(
                            cv::countNonZero(goalEdges(topBand))
                        ) / static_cast<double>(topBand.area());

                        const int belowY = bottom + 1;
                        const int belowHeight = std::min(
                            std::max(4, image.rows / 12),
                            image.rows - belowY
                        );
                        double greenBelowRatio = 0.0;
                        if (belowHeight > 0) {
                            const cv::Rect belowRect(
                                left.x,
                                belowY,
                                separation + 1,
                                belowHeight
                            );
                            greenBelowRatio = static_cast<double>(
                                cv::countNonZero(goalGreenMask(belowRect))
                            ) / static_cast<double>(belowRect.area());
                        }
                        if (topEdgeRatio < 0.025 ||
                            greenBelowRatio < 0.15) {
                            continue;
                        }

                        const double score =
                            200.0 +
                            static_cast<double>(separation) / image.cols *
                                4.0 +
                            static_cast<double>(left.bottom - left.top +
                                right.bottom - right.top) / image.rows +
                            topEdgeRatio + greenBelowRatio;
                        if (score > bestGoalScore) {
                            bestGoalScore = score;
                            bestGoalRect = cv::Rect(
                                left.x,
                                top,
                                separation + 1,
                                height
                            ) & fullImageRect;
                            foundGoalThisFrame = true;
                            t1GoalDetectionMethod = 1;
                        }
                    }
                }
            }

            // 主检测：分别保留长横向白结构和长竖向白结构。真正的球门
            // 必须同时具有一条上横梁，以及横梁两端向下延伸的立柱。
            // 这比把门网、门柱和场地线强行当成一个外轮廓更稳定。
            cv::Mat horizontalGoalMask;
            cv::Mat verticalGoalMask;
            const cv::Mat horizontalGoalKernel =
                cv::getStructuringElement(
                    cv::MORPH_RECT,
                    cv::Size(std::max(25, image.cols / 8), 3)
                );
            const cv::Mat verticalGoalKernel =
                cv::getStructuringElement(
                    cv::MORPH_RECT,
                    cv::Size(3, std::max(20, image.rows / 10))
                );
            cv::morphologyEx(
                goalWhiteMask,
                horizontalGoalMask,
                cv::MORPH_OPEN,
                horizontalGoalKernel
            );
            cv::morphologyEx(
                goalWhiteMask,
                verticalGoalMask,
                cv::MORPH_OPEN,
                verticalGoalKernel
            );

            std::vector<std::vector<cv::Point>> crossbarContours;
            cv::findContours(
                horizontalGoalMask,
                crossbarContours,
                cv::RETR_EXTERNAL,
                cv::CHAIN_APPROX_SIMPLE
            );
            for (const auto &crossbarContour : crossbarContours) {
                const cv::Rect crossbarRect =
                    cv::boundingRect(crossbarContour) & fullImageRect;
                if (crossbarRect.width < image.cols * 0.18 ||
                    crossbarRect.width > image.cols * 0.70 ||
                    crossbarRect.y < image.rows * 0.12 ||
                    crossbarRect.y > image.rows * 0.48 ||
                    crossbarRect.height > image.rows * 0.10) {
                    continue;
                }

                const int sideSearchWidth = std::max(
                    10,
                    crossbarRect.width / 7
                );
                const int searchTop = std::max(
                    0,
                    crossbarRect.y - image.rows / 40
                );
                const int searchBottom = std::min(
                    image.rows,
                    crossbarRect.y + image.rows * 2 / 5
                );
                const int searchHeight = searchBottom - searchTop;
                const cv::Rect leftSearch(
                    std::max(0, crossbarRect.x - sideSearchWidth / 3),
                    searchTop,
                    std::min(
                        sideSearchWidth,
                        image.cols - std::max(
                            0,
                            crossbarRect.x - sideSearchWidth / 3
                        )
                    ),
                    searchHeight
                );
                const int rightSearchX = std::max(
                    0,
                    std::min(
                        image.cols - 1,
                        crossbarRect.x + crossbarRect.width -
                            sideSearchWidth * 2 / 3
                    )
                );
                const cv::Rect rightSearch(
                    rightSearchX,
                    searchTop,
                    std::min(sideSearchWidth, image.cols - rightSearchX),
                    searchHeight
                );
                if (leftSearch.area() <= 0 || rightSearch.area() <= 0) {
                    continue;
                }

                std::vector<cv::Point> leftPostPoints;
                std::vector<cv::Point> rightPostPoints;
                cv::findNonZero(
                    verticalGoalMask(leftSearch),
                    leftPostPoints
                );
                cv::findNonZero(
                    verticalGoalMask(rightSearch),
                    rightPostPoints
                );
                if (leftPostPoints.empty() || rightPostPoints.empty()) {
                    continue;
                }
                cv::Rect leftPostRect = cv::boundingRect(leftPostPoints);
                cv::Rect rightPostRect = cv::boundingRect(rightPostPoints);
                leftPostRect.x += leftSearch.x;
                leftPostRect.y += leftSearch.y;
                rightPostRect.x += rightSearch.x;
                rightPostRect.y += rightSearch.y;
                if (leftPostRect.height < image.rows * 0.08 ||
                    rightPostRect.height < image.rows * 0.08) {
                    continue;
                }

                const int goalLeft = std::min(
                    crossbarRect.x,
                    leftPostRect.x
                );
                const int goalRight = std::max(
                    crossbarRect.x + crossbarRect.width,
                    rightPostRect.x + rightPostRect.width
                );
                const int goalTop = std::min(
                    crossbarRect.y,
                    std::min(leftPostRect.y, rightPostRect.y)
                );
                const int goalBottom = std::max(
                    leftPostRect.y + leftPostRect.height,
                    rightPostRect.y + rightPostRect.height
                );
                const cv::Rect structuredGoalRect(
                    goalLeft,
                    goalTop,
                    goalRight - goalLeft,
                    goalBottom - goalTop
                );
                if (structuredGoalRect.height < image.rows * 0.08 ||
                    structuredGoalRect.width <
                        structuredGoalRect.height * 1.4 ||
                    structuredGoalRect.width >
                        structuredGoalRect.height * 4.2) {
                    continue;
                }

                const double score =
                    100.0 +
                    static_cast<double>(structuredGoalRect.width) /
                        image.cols * 4.0 +
                    static_cast<double>(leftPostRect.height +
                        rightPostRect.height) / image.rows;
                if (score > bestGoalScore) {
                    bestGoalScore = score;
                    bestGoalRect = structuredGoalRect & fullImageRect;
                    foundGoalThisFrame = true;
                    t1GoalDetectionMethod = 2;
                }
            }

            std::vector<std::vector<cv::Point>> goalContours;
            cv::Mat goalContourMask = goalWhiteMask.clone();
            cv::findContours(
                goalContourMask,
                goalContours,
                cv::RETR_EXTERNAL,
                cv::CHAIN_APPROX_SIMPLE
            );

            for (const auto &contour : goalContours) {
                cv::Rect rect = cv::boundingRect(contour) & fullImageRect;
                if (rect.width < image.cols * 0.18 ||
                    rect.width > image.cols * 0.70 ||
                    rect.height < image.rows * 0.08 ||
                    rect.height > image.rows * 0.55 ||
                    rect.width < rect.height * 1.4 ||
                    rect.width > rect.height * 4.2 ||
                    rect.x <= image.cols * 0.03 ||
                    rect.x + rect.width >= image.cols * 0.97 ||
                    rect.y + rect.height / 2 > image.rows * 0.62) {
                    continue;
                }

                const int topHeight = std::max(3, rect.height / 5);
                const int sideWidth = std::max(3, rect.width / 12);
                const cv::Rect topStrip(
                    rect.x,
                    rect.y,
                    rect.width,
                    std::min(topHeight, rect.height)
                );
                const cv::Rect leftStrip(
                    rect.x,
                    rect.y,
                    std::min(sideWidth, rect.width),
                    rect.height
                );
                const cv::Rect rightStrip(
                    rect.x + rect.width - std::min(sideWidth, rect.width),
                    rect.y,
                    std::min(sideWidth, rect.width),
                    rect.height
                );
                const double topRatio = static_cast<double>(
                    cv::countNonZero(goalWhiteMask(topStrip))
                ) / static_cast<double>(topStrip.area());
                const double leftRatio = static_cast<double>(
                    cv::countNonZero(goalWhiteMask(leftStrip))
                ) / static_cast<double>(leftStrip.area());
                const double rightRatio = static_cast<double>(
                    cv::countNonZero(goalWhiteMask(rightStrip))
                ) / static_cast<double>(rightStrip.area());

                const int belowY = rect.y + rect.height;
                const int belowHeight = std::min(
                    std::max(4, image.rows / 12),
                    image.rows - belowY
                );
                double greenBelowRatio = 0.0;
                if (belowHeight > 0) {
                    const cv::Rect belowRect(
                        rect.x,
                        belowY,
                        rect.width,
                        belowHeight
                    );
                    greenBelowRatio = static_cast<double>(
                        cv::countNonZero(goalGreenMask(belowRect))
                    ) / static_cast<double>(belowRect.area());
                }

                if (topRatio < 0.08 || leftRatio < 0.05 ||
                    rightRatio < 0.05 || greenBelowRatio < 0.15) {
                    continue;
                }
                const double score =
                    static_cast<double>(rect.width) / image.cols * 4.0 +
                    topRatio + leftRatio + rightRatio + greenBelowRatio;
                if (score > bestGoalScore) {
                    bestGoalScore = score;
                    bestGoalRect = rect;
                    foundGoalThisFrame = true;
                    t1GoalDetectionMethod = 3;
                }
            }

            // 门框可能与地面白线在掩膜中连成一个超大轮廓。此时改用
            // 两条长竖线加顶部横梁证据，不依赖轮廓彼此分离。
            if (!foundGoalThisFrame) {
                std::vector<cv::Vec4i> whiteLines;
                cv::HoughLinesP(
                    goalWhiteMask,
                    whiteLines,
                    1.0,
                    CV_PI / 180.0,
                    24,
                    image.rows * 0.08,
                    10.0
                );
                struct VerticalGoalLine {
                    int x;
                    int top;
                    int bottom;
                };
                std::vector<VerticalGoalLine> verticalLines;
                for (const auto &line : whiteLines) {
                    const int dx = line[2] - line[0];
                    const int dy = line[3] - line[1];
                    if (std::abs(dy) < image.rows * 0.08 ||
                        std::abs(dx) > std::abs(dy) * 0.25) {
                        continue;
                    }
                    const int top = std::min(line[1], line[3]);
                    const int bottom = std::max(line[1], line[3]);
                    if ((top + bottom) / 2 > image.rows * 0.62) {
                        continue;
                    }
                    verticalLines.push_back({
                        (line[0] + line[2]) / 2,
                        top,
                        bottom
                    });
                }

                for (std::size_t i = 0; i < verticalLines.size(); ++i) {
                    for (std::size_t j = i + 1;
                         j < verticalLines.size(); ++j) {
                        const VerticalGoalLine &first = verticalLines[i];
                        const VerticalGoalLine &second = verticalLines[j];
                        const VerticalGoalLine &left =
                            first.x <= second.x ? first : second;
                        const VerticalGoalLine &right =
                            first.x <= second.x ? second : first;
                        const int separation = right.x - left.x;
                        if (separation < image.cols * 0.18 ||
                            separation > image.cols * 0.70 ||
                            left.x <= image.cols * 0.03 ||
                            right.x >= image.cols * 0.97 ||
                            std::abs(left.top - right.top) >
                                image.rows * 0.12) {
                            continue;
                        }
                        const int top = std::max(
                            0,
                            std::min(left.top, right.top)
                        );
                        const int bottom = std::min(
                            image.rows - 1,
                            std::max(left.bottom, right.bottom)
                        );
                        const int height = bottom - top + 1;
                        if (height < image.rows * 0.08 ||
                            separation < height * 1.4 ||
                            separation > height * 4.2) {
                            continue;
                        }
                        const cv::Rect pairRect(
                            left.x,
                            top,
                            separation + 1,
                            height
                        );
                        const int topBandHeight = std::min(
                            std::max(4, image.rows / 30),
                            pairRect.height
                        );
                        const cv::Rect topBand(
                            pairRect.x,
                            pairRect.y,
                            pairRect.width,
                            topBandHeight
                        );
                        const double topRatio = static_cast<double>(
                            cv::countNonZero(goalWhiteMask(topBand))
                        ) / static_cast<double>(topBand.area());

                        const int belowY = pairRect.y + pairRect.height;
                        const int belowHeight = std::min(
                            std::max(4, image.rows / 12),
                            image.rows - belowY
                        );
                        double greenBelowRatio = 0.0;
                        if (belowHeight > 0) {
                            const cv::Rect belowRect(
                                pairRect.x,
                                belowY,
                                pairRect.width,
                                belowHeight
                            );
                            greenBelowRatio = static_cast<double>(
                                cv::countNonZero(goalGreenMask(belowRect))
                            ) / static_cast<double>(belowRect.area());
                        }
                        if (topRatio < 0.04 || greenBelowRatio < 0.15) {
                            continue;
                        }
                        const double score =
                            static_cast<double>(separation) / image.cols * 5.0 +
                            static_cast<double>(height) / image.rows +
                            topRatio + greenBelowRatio;
                        if (score > bestGoalScore) {
                            bestGoalScore = score;
                            bestGoalRect = pairRect;
                            foundGoalThisFrame = true;
                            t1GoalDetectionMethod = 4;
                        }
                    }
                }
            }

            if (foundGoalThisFrame) {
                const cv::Point newCenter(
                    bestGoalRect.x + bestGoalRect.width / 2,
                    bestGoalRect.y + bestGoalRect.height / 2
                );
                const cv::Point oldCenter(
                    trackedGoalRect.x + trackedGoalRect.width / 2,
                    trackedGoalRect.y + trackedGoalRect.height / 2
                );
                const cv::Point anchorCenter(
                    t1KeeperAnchorRect.x + t1KeeperAnchorRect.width / 2,
                    t1KeeperAnchorRect.y + t1KeeperAnchorRect.height / 2
                );
                const cv::Rect overlap = bestGoalRect & trackedGoalRect;
                const double smallerArea = static_cast<double>(
                    std::max(
                        1,
                        std::min(
                            bestGoalRect.area(),
                            trackedGoalRect.area()
                        )
                    )
                );
                const bool agreesWithTrack = hasGoalTrack &&
                    cv::norm(newCenter - oldCenter) <= image.cols * 0.15 &&
                    std::abs(bestGoalRect.width - trackedGoalRect.width) <=
                        image.cols * 0.25 &&
                    std::abs(bestGoalRect.height - trackedGoalRect.height) <=
                        image.rows * 0.20 &&
                    static_cast<double>(overlap.area()) / smallerArea >= 0.25 &&
                    (!t1KeeperAnchorVisible ||
                        (bestGoalRect.contains(anchorCenter) &&
                         trackedGoalRect.contains(anchorCenter)));
                goalStableFrames = agreesWithTrack
                    ? std::min(goalStableFrames + 1, goalConfirmFrames)
                    : 1;
                if (agreesWithTrack) {
                    // 门柱内外沿会让 Hough 框轻微跳动；低通更新框，
                    // 保持候选射门点连续，不追逐单帧边缘。
                    trackedGoalRect = cv::Rect(
                        (trackedGoalRect.x * 3 + bestGoalRect.x) / 4,
                        (trackedGoalRect.y * 3 + bestGoalRect.y) / 4,
                        (trackedGoalRect.width * 3 +
                            bestGoalRect.width) / 4,
                        (trackedGoalRect.height * 3 +
                            bestGoalRect.height) / 4
                    ) & fullImageRect;
                } else {
                    trackedGoalRect = bestGoalRect;
                }
                hasGoalTrack = true;
                goalMissFrames = 0;
            } else if (hasGoalTrack) {
                ++goalMissFrames;
                const cv::Point anchorCenter(
                    t1KeeperAnchorRect.x + t1KeeperAnchorRect.width / 2,
                    t1KeeperAnchorRect.y + t1KeeperAnchorRect.height / 2
                );
                const bool stableAnchorKeepsGoal =
                    t1KeeperAnchorVisible &&
                    trackedGoalRect.contains(anchorCenter);
                if (stableAnchorKeepsGoal) {
                    // 首帧已由横梁和双立柱确认；静止画面中边缘短暂
                    // 断裂时，由仍处于框内的门将锚点维持连续性。
                    goalStableFrames = std::min(
                        goalStableFrames + 1,
                        goalConfirmFrames
                    );
                    t1GoalDetectionMethod = 5;
                }
                const int allowedGoalMissFrames =
                    stableAnchorKeepsGoal ? 10 : 8;
                if (goalMissFrames >= allowedGoalMissFrames) {
                    hasGoalTrack = false;
                    trackedGoalRect = cv::Rect();
                    goalStableFrames = 0;
                    goalMissFrames = 0;
                }
            }

            bool foundKeeperThisFrame = false;
            cv::Rect bestKeeperRect;
            double bestKeeperArea = 0.0;
            if (hasGoalTrack && t1KeeperAnchorVisible) {
                const cv::Point anchorCenter(
                    t1KeeperAnchorRect.x + t1KeeperAnchorRect.width / 2,
                    t1KeeperAnchorRect.y + t1KeeperAnchorRect.height / 2
                );
                if (trackedGoalRect.contains(anchorCenter)) {
                    bestKeeperRect = t1KeeperAnchorRect;
                    bestKeeperArea = static_cast<double>(
                        t1KeeperAnchorRect.area()
                    );
                    foundKeeperThisFrame = true;
                }
            }
            if (hasGoalTrack && !foundKeeperThisFrame) {
                cv::Mat blueMask = keeperColorMask.clone();
                std::vector<std::vector<cv::Point>> keeperContours;
                cv::findContours(
                    blueMask,
                    keeperContours,
                    cv::RETR_EXTERNAL,
                    cv::CHAIN_APPROX_SIMPLE
                );
                for (const auto &contour : keeperContours) {
                    const double area = cv::contourArea(contour);
                    cv::Rect rect = cv::boundingRect(contour) & fullImageRect;
                    const cv::Point center(
                        rect.x + rect.width / 2,
                        rect.y + rect.height / 2
                    );
                    if (area < 8.0 ||
                        rect.width > trackedGoalRect.width * 0.35 ||
                        rect.height > trackedGoalRect.height * 0.90 ||
                        center.y < trackedGoalRect.y +
                            trackedGoalRect.height / 4 ||
                        !trackedGoalRect.contains(center)) {
                        continue;
                    }
                    if (area > bestKeeperArea) {
                        bestKeeperArea = area;
                        bestKeeperRect = rect;
                        foundKeeperThisFrame = true;
                    }
                }
            }

            if (foundKeeperThisFrame) {
                const cv::Point newCenter(
                    bestKeeperRect.x + bestKeeperRect.width / 2,
                    bestKeeperRect.y + bestKeeperRect.height / 2
                );
                const cv::Point oldCenter(
                    trackedKeeperRect.x + trackedKeeperRect.width / 2,
                    trackedKeeperRect.y + trackedKeeperRect.height / 2
                );
                const bool agreesWithTrack = hasKeeperTrack &&
                    cv::norm(newCenter - oldCenter) <= image.cols * 0.06;
                keeperStableFrames = agreesWithTrack
                    ? std::min(keeperStableFrames + 1, keeperConfirmFrames)
                    : 1;
                trackedKeeperRect = bestKeeperRect;
                hasKeeperTrack = true;
                keeperMissFrames = 0;
            } else if (hasKeeperTrack) {
                ++keeperMissFrames;
                if (keeperMissFrames >= 8) {
                    hasKeeperTrack = false;
                    trackedKeeperRect = cv::Rect();
                    keeperStableFrames = 0;
                    keeperMissFrames = 0;
                }
            } else {
                keeperStableFrames = 0;
            }

            // T2：球门首次连续确认后立即冻结几何。机器人尚未移动时，
            // 门框在真实场景中不会移动，因此后续检测抖动不应拖动射门点。
            if (!hasFrozenShotGoal && hasGoalTrack &&
                goalStableFrames >= goalConfirmFrames &&
                trackedGoalRect.width > 0) {
                hasFrozenShotGoal = true;
                frozenShotGoalRect = trackedGoalRect;
                shotDecisionWaitFrames = 0;
                keeperMotionReady = false;
                keeperMotionSamples = 0;
                pendingShotSide = 0;
                shotSideStableFrames = 0;
            }

            // T2 只做一次快速决策。门将明显偏离中央时射向反侧；门将
            // 居中或在有限等待内不可见时使用固定默认侧。这里不预测球
            // 出脚后的追球运动，因为示例门将会根据足球方向实时横移。
            if (hasFrozenShotGoal && lockedShotSide == 0) {
                ++shotDecisionWaitFrames;
            }
            if (hasFrozenShotGoal && lockedShotSide == 0 &&
                foundKeeperThisFrame && frozenShotGoalRect.width > 0) {
                const double measuredKeeperU = std::max(
                    0.0,
                    std::min(
                        1.0,
                        (bestKeeperRect.x + bestKeeperRect.width * 0.5 -
                            frozenShotGoalRect.x) /
                            static_cast<double>(frozenShotGoalRect.width)
                    )
                );
                if (!keeperMotionReady) {
                    keeperMotionReady = true;
                    filteredKeeperU = measuredKeeperU;
                    keeperMotionSamples = 1;
                } else {
                    filteredKeeperU = filteredKeeperU * 0.65 +
                        measuredKeeperU * 0.35;
                    keeperMotionSamples = std::min(
                        keeperMotionSamples + 1,
                        1000
                    );
                }

                const double keeperHalfWidthU =
                    bestKeeperRect.width * 0.5 /
                    static_cast<double>(frozenShotGoalRect.width);
                leftShotClearance =
                    std::abs(leftShotTargetU - filteredKeeperU) -
                        keeperHalfWidthU;
                rightShotClearance =
                    std::abs(rightShotTargetU - filteredKeeperU) -
                        keeperHalfWidthU;

                int preferredSide = defaultShotSide;
                if (filteredKeeperU < 0.5 - keeperCenterDeadbandU) {
                    preferredSide = 1;
                } else if (filteredKeeperU >
                    0.5 + keeperCenterDeadbandU) {
                    preferredSide = -1;
                }
                if (preferredSide == pendingShotSide) {
                    shotSideStableFrames = std::min(
                        shotSideStableFrames + 1,
                        shotSideConfirmFrames
                    );
                } else {
                    pendingShotSide = preferredSide;
                    shotSideStableFrames = 1;
                }
                if (shotSideStableFrames >= shotSideConfirmFrames) {
                    lockedShotSide = pendingShotSide;
                }
            }
            if (hasFrozenShotGoal && lockedShotSide == 0 &&
                shotDecisionWaitFrames >= shotDecisionMaxWaitFrames) {
                pendingShotSide = defaultShotSide;
                shotSideStableFrames = shotSideConfirmFrames;
                lockedShotSide = defaultShotSide;
            }

            if (lockedShotSide != 0 && hasFrozenShotGoal &&
                approachPhase == ApproachPhase::GOAL_VIEW_TEST) {
                const double selectedGoalU = lockedShotSide < 0
                    ? leftShotTargetU
                    : rightShotTargetU;
                const double targetPixelX = frozenShotGoalRect.x +
                    frozenShotGoalRect.width * selectedGoalU;
                const double cameraCenterX = image.cols * 0.5;
                const double cameraFocalPixels = image.cols /
                    (2.0 * std::tan(cameraHorizontalFieldOfView * 0.5));
                shotTargetVisualAngleDegrees = std::atan(
                    (cameraCenterX - targetPixelX) / cameraFocalPixels
                ) * radiansToDegrees;
                shotTargetAimAngleDegrees =
                    shotTargetVisualAngleDegrees +
                    rightFootKickAimCompensationDegrees;
                shotTargetCorrectionPulses = 0;
                shotTargetAlignmentSafetyStop =
                    !imuTurnDirectionKnown ||
                    !std::isfinite(shotTargetAimAngleDegrees) ||
                    std::abs(shotTargetAimAngleDegrees) >
                        maxShotTargetVisualAngleDegrees;
                headTargetYaw = 0.0;
                headTargetPitch = 0.0;
                htask.yaw = headTargetYaw;
                htask.pitch = headTargetPitch;

                if (shotTargetAlignmentSafetyStop) {
                    shotTargetPhysicalError =
                        shotTargetAimAngleDegrees;
                    approachPhase = ApproachPhase::SHOT_TARGET_STOP;
                } else {
                    shotTargetImuYaw = normalizeAngle(
                        static_cast<double>(imuData.yaw) +
                        imuYawDirectionForPositiveTurn *
                            shotTargetAimAngleDegrees
                    );
                    shotTargetPhysicalError =
                        shotTargetAimAngleDegrees;
                    activeTurnCommand = shotTargetAimAngleDegrees /
                        static_cast<double>(turnCycleCount);
                    approachPhase = ApproachPhase::SHOT_TARGET_TURN;
                    approachPhaseFrames = turnCommandFrames;
                    enteredShotTargetTurnThisFrame = true;
                }
            }
        }

        // T3：平视状态下身体转向冻结射门点，IMU 停稳复核；
        // 本阶段不低头、不靠近足球、不踢球。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright && !enteredShotTargetTurnThisFrame &&
            (approachPhase == ApproachPhase::SHOT_TARGET_TURN ||
             approachPhase == ApproachPhase::SHOT_TARGET_SETTLE)) {
            if (approachPhaseFrames > 0) {
                approachPhaseFrames = std::max(
                    0,
                    approachPhaseFrames - motionFrameAdvance
                );
            }
            if (approachPhaseFrames == 0) {
                if (approachPhase == ApproachPhase::SHOT_TARGET_TURN) {
                    approachPhase = ApproachPhase::SHOT_TARGET_SETTLE;
                    approachPhaseFrames = precisionTurnSettleFrames;
                    btask.step = 0.0;
                    btask.lateral = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else {
                    shotTargetPhysicalError = normalizeAngle(
                        shotTargetImuYaw -
                        static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    if (std::abs(shotTargetPhysicalError) >
                            shotTargetYawAcceptDegrees &&
                        shotTargetCorrectionPulses <
                            maxShotTargetCorrectionPulses) {
                        activeTurnCommand = std::max(
                            -maxShotTargetCorrectionPerCycle,
                            std::min(
                                maxShotTargetCorrectionPerCycle,
                                shotTargetPhysicalError /
                                    static_cast<double>(turnCycleCount)
                            )
                        );
                        ++shotTargetCorrectionPulses;
                        approachPhase = ApproachPhase::SHOT_TARGET_TURN;
                        approachPhaseFrames = turnCommandFrames;
                        enteredShotTargetTurnThisFrame = true;
                    } else {
                        shotTargetAlignmentSafetyStop =
                            std::abs(shotTargetPhysicalError) >
                                shotTargetYawAcceptDegrees;
                        btask.step = 0.0;
                        btask.lateral = 0.0;
                        btask.turn = 0.0;
                        btask.count = 0;
                        if (shotTargetAlignmentSafetyStop) {
                            approachPhase = ApproachPhase::SHOT_TARGET_STOP;
                        } else {
                            // T3 已通过 IMU 复核。T4 只移动头部并在身体
                            // 完全静止时重捕获足球；确认前禁止任何位移。
                            approachPhase = ApproachPhase::BALL_REACQUIRE;
                            hasBallTrack = false;
                            trackedBallCenter = cv::Point();
                            trackedBallRect = cv::Rect();
                            lostBallFrames = 0;
                            settledLostBallFrames = 0;
                            stableBallFrames = 0;
                            liveBallFrames = 0;
                            localReacquirePoseIndex = 0;
                            reacquireAfterTurn = true;
                            reacquireMatchFrames = 0;
                            reacquireMissFrames = 0;
                            nearBallGeometrySamples = 0;
                            nearBallCenterXSum = 0.0;
                            nearBallCenterYSum = 0.0;
                            nearBallRadiusSum = 0.0;
                            nearBallConfirmedRadius = 0;
                            ballReacquireSucceeded = false;
                            ballReacquireSafetyStop = false;
                            headTargetYaw =
                                localReacquirePoses.front().yaw;
                            headTargetPitch =
                                localReacquirePoses.front().pitch;
                            htask.yaw = headTargetYaw;
                            htask.pitch = headTargetPitch;
                        }
                    }
                }
            }
        }

        const auto beginCoarseBallReacquire = [&]() {
            approachPhase = ApproachPhase::COARSE_BALL_REACQUIRE;
            hasBallTrack = false;
            trackedBallCenter = cv::Point();
            trackedBallRect = cv::Rect();
            lostBallFrames = 0;
            settledLostBallFrames = 0;
            stableBallFrames = 0;
            liveBallFrames = 0;
            reacquireAfterTurn = true;
            reacquireMatchFrames = 0;
            reacquireMissFrames = 0;
            nearBallGeometrySamples = 0;
            nearBallCenterXSum = 0.0;
            nearBallCenterYSum = 0.0;
            nearBallRadiusSum = 0.0;
            nearBallConfirmedRadius = 0;
            headTargetYaw = ballReacquireHeadYaw;
            headTargetPitch = ballReacquireHeadPitch;
            htask.yaw = headTargetYaw;
            htask.pitch = headTargetPitch;
        };

        const auto beginForwardBallReacquire = [&]() {
            approachPhase = ApproachPhase::FORWARD_BALL_REACQUIRE;
            hasBallTrack = false;
            trackedBallCenter = cv::Point();
            trackedBallRect = cv::Rect();
            lostBallFrames = 0;
            settledLostBallFrames = 0;
            stableBallFrames = 0;
            liveBallFrames = 0;
            reacquireAfterTurn = true;
            reacquireMatchFrames = 0;
            reacquireMissFrames = 0;
            nearBallGeometrySamples = 0;
            nearBallCenterXSum = 0.0;
            nearBallCenterYSum = 0.0;
            nearBallRadiusSum = 0.0;
            nearBallConfirmedRadius = 0;
            headTargetYaw = ballReacquireHeadYaw;
            headTargetPitch = ballReacquireHeadPitch;
            htask.yaw = headTargetYaw;
            htask.pitch = headTargetPitch;
        };

        const auto beginFineBallReacquire = [&]() {
            approachPhase = ApproachPhase::FINE_BALL_REACQUIRE;
            hasBallTrack = false;
            trackedBallCenter = cv::Point();
            trackedBallRect = cv::Rect();
            lostBallFrames = 0;
            settledLostBallFrames = 0;
            stableBallFrames = 0;
            liveBallFrames = 0;
            reacquireAfterTurn = true;
            reacquireMatchFrames = 0;
            reacquireMissFrames = 0;
            nearBallGeometrySamples = 0;
            nearBallCenterXSum = 0.0;
            nearBallCenterYSum = 0.0;
            nearBallRadiusSum = 0.0;
            nearBallConfirmedRadius = 0;
            headTargetYaw = ballReacquireHeadYaw;
            headTargetPitch = ballReacquireHeadPitch;
            htask.yaw = headTargetYaw;
            htask.pitch = headTargetPitch;
        };

        // 同一幅针孔图像中 (球心横偏/球半径) = (实际横偏/球半径)，
        // 因而不必猜随距离变化的“像素/米”增益。T5 实测的
        // lateral=0.015,count=2 约产生 0.03m 位移，故每周期取
        // 剩余横差的一半，再按 walk.conf 的 0.03m 上限保守限幅。
        const auto lateralCommandForFootWindow = [&](int ballX,
            int ballRadius) {
            const int pixelError = ballX - preferredFootTargetX;
            if (pixelError == 0 || ballRadius <= 0) {
                return 0.0;
            }
            const double estimatedCommand =
                -0.5 * 0.07 * static_cast<double>(pixelError) /
                    static_cast<double>(ballRadius);
            const double magnitude = std::max(
                0.006,
                std::min(0.025, std::abs(estimatedCommand))
            );
            return estimatedCommand > 0.0 ? magnitude : -magnitude;
        };

        const auto evaluateKickGeometry = [&]() {
            finalBallX = trackedBallCenter.x;
            finalBallY = trackedBallCenter.y;
            finalBallRadius = nearBallConfirmedRadius;
            finalHeadingError = normalizeAngle(
                shotTargetImuYaw - static_cast<double>(imuData.yaw)
            ) * imuYawDirectionForPositiveTurn;

            if (std::abs(finalHeadingError) > fineYawAcceptDegrees) {
                if (fineYawCorrectionPulses <
                    maxFineYawCorrectionPulses) {
                    activeTurnCommand = std::max(
                        -maxCoarseYawCorrectionPerCycle,
                        std::min(
                            maxCoarseYawCorrectionPerCycle,
                            finalHeadingError /
                                static_cast<double>(turnCycleCount)
                        )
                    );
                    ++fineYawCorrectionPulses;
                    approachPhase = ApproachPhase::FINE_YAW_RECOVER;
                    approachPhaseFrames = turnCommandFrames;
                } else {
                    fineSafetyStop = true;
                    fineStopReason = "yaw correction limit";
                    approachPhase = ApproachPhase::KICK_STOP;
                }
                return;
            }
            // 这一批运动后的绝对航向已经达标，下一批动作重新获得
            // 有界修正预算；总动作次数仍由前进/横移上限约束。
            fineYawCorrectionPulses = 0;

            // 45° 视角下球变大后会先触及画面下沿。靠近到约
            // 60px 时提前低头，静止重捕获后再决定下一步；否则
            // “球完整可见”的安全门会在脚前距离之前误停。
            if (finalBallRadius >= 60 &&
                ballReacquireHeadPitch < kickCloseHeadPitch - 3.0) {
                ballReacquireHeadYaw = 0.0;
                ballReacquireHeadPitch = kickCloseHeadPitch;
                beginFineBallReacquire();
                return;
            }

            const bool ballFullyVisible =
                finalBallY - finalBallRadius >= 2 &&
                finalBallY + finalBallRadius < image.rows - 2;
            const bool longitudinalReady =
                finalBallRadius >= kickBallRadiusMin &&
                finalBallRadius <= kickBallRadiusMax;
            const bool ballTooClose =
                finalBallRadius > kickBallRadiusSafetyMax;
            const bool ballClearlyFar =
                finalBallRadius < kickBallRadiusMin;
            const int lateralError =
                finalBallX - preferredFootTargetX;
            const bool lateralReady =
                std::abs(lateralError) <= preferredFootTolerancePixels;
            // 落点横向误差较大时先修横向，避免沿错误的射门线逼近
            // 足球；误差缩小后再前进，最后在脚前窗口做精横移。
            const bool lateralPriority = !lateralReady &&
                (longitudinalReady ||
                    std::abs(lateralError) > std::max(
                        preferredFootTolerancePixels * 2,
                        cvRound(finalBallRadius * 0.45)
                    ));

            if (!ballFullyVisible || ballTooClose ||
                (!longitudinalReady && !ballClearlyFar) ||
                (!longitudinalReady &&
                    fineForwardPulses >= maxFineForwardPulses) ||
                (!lateralReady &&
                    fineLateralPulses >= maxFineLateralPulses)) {
                fineSafetyStop = true;
                fineStopReason = !ballFullyVisible
                    ? "ball clipped"
                    : ballTooClose
                        ? "ball too close"
                        : !longitudinalReady && !ballClearlyFar
                            ? "forward overshot"
                            : !longitudinalReady
                                ? "forward pulse limit"
                                : "lateral pulse limit";
                approachPhase = ApproachPhase::KICK_STOP;
                return;
            }

            if (!longitudinalReady || !lateralReady) {
                fineStepCommand = 0.0;
                fineLateralCommand = 0.0;

                // 偏差大时先横移。球的角尺寸和横偏共享同一透视
                // 比例，因此复用经过 T5 方向验证的物理尺度命令。
                if (lateralPriority) {
                    fineLateralCommand =
                        lateralCommandForFootWindow(
                            finalBallX,
                            finalBallRadius
                        );
                    ++fineLateralPulses;
                } else if (!longitudinalReady) {
                    // 用球的角尺寸估计剩余物理距离。count=2，
                    // 单周期命令取预计位移的一半；每批后复测。
                    const double focalPixels = image.cols /
                        (2.0 * std::tan(
                            cameraHorizontalFieldOfView * 0.5));
                    const double targetRadius =
                        (kickBallRadiusMin + kickBallRadiusMax) * 0.5;
                    const double estimatedStep =
                        0.5 * focalPixels * 0.07 *
                        (1.0 / finalBallRadius - 1.0 / targetRadius);
                    // 远处仍有足够避球余量，允许低于已验证长走
                    // 0.05m/周期的较大步长；近处自动收小。
                    const double maxStep = finalBallRadius < 60
                        ? 0.040
                        : 0.025;
                    fineStepCommand = std::max(
                        0.006,
                        std::min(maxStep, estimatedStep)
                    );
                    ++fineForwardPulses;
                } else if (!lateralReady) {
                    fineLateralCommand =
                        lateralCommandForFootWindow(
                            finalBallX,
                            finalBallRadius
                        );
                    ++fineLateralPulses;
                }

                approachPhase = ApproachPhase::FINE_MOVE;
                approachPhaseFrames = fineMotionCommandFrames;
                return;
            }

            // 五个连续静止帧已经同时确认二维脚前窗口。锁存后只请求
            // 一次右脚动作，后续状态绝不重新进入本分支。
            kickIssued = true;
            kickWaitCompleted = false;
            kickFallObserved = false;
            kickStartRemainTime = gameData.remain_time;
            approachPhase = ApproachPhase::KICK_REQUEST;
            approachPhaseFrames = kickRequestFrames;
        };

        // T5 的动作严格串行：一次横移、完全停稳、单独恢复绝对射门
        // 航向，再回到同一头姿复测。横移与转向绝不在同一 BodyTask 中。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright &&
            (approachPhase == ApproachPhase::COARSE_LATERAL_MOVE ||
             approachPhase == ApproachPhase::COARSE_LATERAL_SETTLE ||
             approachPhase == ApproachPhase::COARSE_YAW_RECOVER ||
             approachPhase == ApproachPhase::COARSE_YAW_SETTLE)) {
            if (approachPhaseFrames > 0) {
                approachPhaseFrames = std::max(
                    0,
                    approachPhaseFrames - motionFrameAdvance
                );
            }
            if (approachPhaseFrames == 0) {
                if (approachPhase == ApproachPhase::COARSE_LATERAL_MOVE) {
                    approachPhase = ApproachPhase::COARSE_LATERAL_SETTLE;
                    approachPhaseFrames = coarseMotionSettleFrames;
                    btask.step = 0.0;
                    btask.lateral = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else if (approachPhase ==
                    ApproachPhase::COARSE_YAW_RECOVER) {
                    approachPhase = ApproachPhase::COARSE_YAW_SETTLE;
                    approachPhaseFrames = turnSettleFrames;
                    btask.step = 0.0;
                    btask.lateral = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else {
                    coarseHeadingError = normalizeAngle(
                        shotTargetImuYaw -
                            static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    if (std::abs(coarseHeadingError) >
                            coarseYawAcceptDegrees &&
                        coarseYawCorrectionPulses <
                            maxCoarseYawCorrectionPulses) {
                        activeTurnCommand = std::max(
                            -maxCoarseYawCorrectionPerCycle,
                            std::min(
                                maxCoarseYawCorrectionPerCycle,
                                coarseHeadingError /
                                    static_cast<double>(turnCycleCount)
                            )
                        );
                        ++coarseYawCorrectionPulses;
                        approachPhase = ApproachPhase::COARSE_YAW_RECOVER;
                        approachPhaseFrames = turnCommandFrames;
                    } else if (std::abs(coarseHeadingError) >
                        coarseYawAcceptDegrees) {
                        coarseSafetyStop = true;
                        approachPhase = ApproachPhase::COARSE_LATERAL_STOP;
                    } else {
                        beginCoarseBallReacquire();
                    }
                }
            }
        }

        // T6a：只执行一次保守前进脉冲。停稳后先恢复绝对射门航向，
        // 再以 T4/T5 已验证的同一头姿重新测量球心和半径。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright &&
            (approachPhase == ApproachPhase::FORWARD_CALIBRATION_MOVE ||
             approachPhase == ApproachPhase::FORWARD_CALIBRATION_SETTLE ||
             approachPhase == ApproachPhase::FORWARD_YAW_RECOVER ||
             approachPhase == ApproachPhase::FORWARD_YAW_SETTLE)) {
            if (approachPhaseFrames > 0) {
                approachPhaseFrames = std::max(
                    0,
                    approachPhaseFrames - motionFrameAdvance
                );
            }
            if (approachPhaseFrames == 0) {
                if (approachPhase ==
                    ApproachPhase::FORWARD_CALIBRATION_MOVE) {
                    approachPhase =
                        ApproachPhase::FORWARD_CALIBRATION_SETTLE;
                    approachPhaseFrames = forwardCalibrationSettleFrames;
                    btask.step = 0.0;
                    btask.lateral = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else if (approachPhase ==
                    ApproachPhase::FORWARD_YAW_RECOVER) {
                    approachPhase = ApproachPhase::FORWARD_YAW_SETTLE;
                    approachPhaseFrames = turnSettleFrames;
                    btask.step = 0.0;
                    btask.lateral = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else {
                    forwardAfterHeadingError = normalizeAngle(
                        shotTargetImuYaw -
                            static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    if (std::abs(forwardAfterHeadingError) >
                            coarseYawAcceptDegrees &&
                        forwardYawCorrectionPulses <
                            maxForwardYawCorrectionPulses) {
                        activeTurnCommand = std::max(
                            -maxCoarseYawCorrectionPerCycle,
                            std::min(
                                maxCoarseYawCorrectionPerCycle,
                                forwardAfterHeadingError /
                                    static_cast<double>(turnCycleCount)
                            )
                        );
                        ++forwardYawCorrectionPulses;
                        approachPhase =
                            ApproachPhase::FORWARD_YAW_RECOVER;
                        approachPhaseFrames = turnCommandFrames;
                    } else if (std::abs(forwardAfterHeadingError) >
                        coarseYawAcceptDegrees) {
                        forwardSafetyStop = true;
                        approachPhase =
                            ApproachPhase::FORWARD_CALIBRATION_STOP;
                    } else {
                        beginForwardBallReacquire();
                    }
                }
            }
        }

        // 纵向与横向微调每次只修一个轴。每个脉冲后完全停稳，恢复
        // 已加入右脚几何补偿的绝对射门航向，再静止重捕获足球。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright &&
            (approachPhase == ApproachPhase::FINE_MOVE ||
             approachPhase == ApproachPhase::FINE_SETTLE ||
             approachPhase == ApproachPhase::FINE_YAW_RECOVER ||
             approachPhase == ApproachPhase::FINE_YAW_SETTLE)) {
            if (approachPhaseFrames > 0) {
                approachPhaseFrames = std::max(
                    0,
                    approachPhaseFrames - motionFrameAdvance
                );
            }
            if (approachPhaseFrames == 0) {
                if (approachPhase == ApproachPhase::FINE_MOVE) {
                    approachPhase = ApproachPhase::FINE_SETTLE;
                    approachPhaseFrames = fineMotionSettleFrames;
                    btask.step = 0.0;
                    btask.lateral = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else if (approachPhase ==
                    ApproachPhase::FINE_YAW_RECOVER) {
                    approachPhase = ApproachPhase::FINE_YAW_SETTLE;
                    approachPhaseFrames = precisionTurnSettleFrames;
                    btask.step = 0.0;
                    btask.lateral = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else {
                    finalHeadingError = normalizeAngle(
                        shotTargetImuYaw -
                            static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    if (std::abs(finalHeadingError) >
                            fineYawAcceptDegrees &&
                        fineYawCorrectionPulses <
                            maxFineYawCorrectionPulses) {
                        activeTurnCommand = std::max(
                            -maxCoarseYawCorrectionPerCycle,
                            std::min(
                                maxCoarseYawCorrectionPerCycle,
                                finalHeadingError /
                                    static_cast<double>(turnCycleCount)
                            )
                        );
                        ++fineYawCorrectionPulses;
                        approachPhase =
                            ApproachPhase::FINE_YAW_RECOVER;
                        approachPhaseFrames = turnCommandFrames;
                    } else if (std::abs(finalHeadingError) >
                        fineYawAcceptDegrees) {
                        fineSafetyStop = true;
                        fineStopReason = "yaw did not converge";
                        approachPhase = ApproachPhase::KICK_STOP;
                    } else {
                        beginFineBallReacquire();
                    }
                }
            }
        }

        // TASK_ACT 只保持有限的新相机帧，确保 motion 能读取请求；随后
        // 永久恢复零行走并等待完整动作窗口，禁止重复触发 right_kick。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            (approachPhase == ApproachPhase::KICK_REQUEST ||
             approachPhase == ApproachPhase::KICK_WAIT)) {
            if (!robotUpright) {
                kickFallObserved = true;
            }
            if (approachPhaseFrames > 0) {
                approachPhaseFrames = std::max(
                    0,
                    approachPhaseFrames - motionFrameAdvance
                );
            }
            if (approachPhaseFrames == 0) {
                if (approachPhase == ApproachPhase::KICK_REQUEST) {
                    approachPhase = ApproachPhase::KICK_WAIT;
                    approachPhaseFrames = kickWaitFrames;
                    btask.type = btask.TASK_WALK;
                    btask.step = 0.0;
                    btask.lateral = 0.0;
                    btask.turn = 0.0;
                    btask.count = 0;
                } else {
                    kickWaitCompleted = true;
                    kickStopRemainTime = gameData.remain_time;
                    approachPhase = ApproachPhase::KICK_STOP;
                }
            }
        }

        if (gameData.state == gameData.STATE_PLAY &&
            (approachPhase == ApproachPhase::BALL_REACQUIRE ||
             approachPhase == ApproachPhase::COARSE_BALL_REACQUIRE ||
             approachPhase == ApproachPhase::FORWARD_BALL_REACQUIRE ||
             approachPhase == ApproachPhase::FINE_BALL_REACQUIRE) &&
            reacquireAfterTurn) {
            if (reacquireMatchFrames >= reacquireConfirmFrames) {
                const bool initialNearBallObservation =
                    approachPhase == ApproachPhase::BALL_REACQUIRE;
                const bool forwardCalibrationObservation =
                    approachPhase ==
                        ApproachPhase::FORWARD_BALL_REACQUIRE;
                const bool fineAdjustmentObservation =
                    approachPhase == ApproachPhase::FINE_BALL_REACQUIRE;
                const int confirmedSamples = std::max(
                    1,
                    nearBallGeometrySamples
                );
                trackedBallCenter = cv::Point(
                    cvRound(nearBallCenterXSum / confirmedSamples),
                    cvRound(nearBallCenterYSum / confirmedSamples)
                );
                nearBallConfirmedRadius = cvRound(
                    nearBallRadiusSum / confirmedSamples
                );
                trackedBallRect = cv::Rect(
                    trackedBallCenter.x - nearBallConfirmedRadius,
                    trackedBallCenter.y - nearBallConfirmedRadius,
                    nearBallConfirmedRadius * 2,
                    nearBallConfirmedRadius * 2
                ) & cv::Rect(0, 0, image.cols, image.rows);
                reacquireAfterTurn = false;
                reacquireMissFrames = 0;
                ballReacquireSucceeded = true;
                ballReacquireSafetyStop = false;
                ballReacquireHeadYaw =
                    static_cast<double>(headAngle.yaw);
                ballReacquireHeadPitch =
                    static_cast<double>(headAngle.pitch);
                preferredFootTargetX = image.cols / 2 + cvRound(
                    rightFootTargetOffsetBallRadii *
                        nearBallConfirmedRadius
                );
                preferredFootTolerancePixels = std::max(
                    minimumFootWindowTolerancePixels,
                    cvRound(
                        footWindowToleranceBallRadii *
                            nearBallConfirmedRadius
                    )
                );
                if (initialNearBallObservation) {
                    // 回到已验证的 T5 右脚横向窗口；不让未经标定的
                    // 纵向控制器越过 T6a 断点直接接管机器人。
                    coarseInitialBallX = trackedBallCenter.x;
                    coarseBeforeBallX = trackedBallCenter.x;
                    coarseBeforeTargetX = preferredFootTargetX;
                    coarseAfterBallX = trackedBallCenter.x;
                    coarsePixelShift = 0;
                    coarseLateralPulses = 0;
                    coarseYawCorrectionPulses = 0;
                    coarseHeadingError = normalizeAngle(
                        shotTargetImuYaw -
                            static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    coarseDirectionMatched = true;
                    coarseSafetyStop =
                        std::abs(coarseHeadingError) >
                            coarseYawAcceptDegrees;
                    coarseAlignmentReached =
                        std::abs(trackedBallCenter.x -
                            preferredFootTargetX) <=
                            preferredFootTolerancePixels;
                    if (coarseSafetyStop || coarseAlignmentReached) {
                        approachPhase =
                            ApproachPhase::COARSE_LATERAL_STOP;
                    } else {
                        coarseLateralCommand =
                            trackedBallCenter.x > preferredFootTargetX
                                ? -0.015 : 0.015;
                        ++coarseLateralPulses;
                        approachPhase =
                            ApproachPhase::COARSE_LATERAL_MOVE;
                        approachPhaseFrames = coarseMotionCommandFrames;
                    }
                } else if (forwardCalibrationObservation) {
                    forwardAfterBallX = trackedBallCenter.x;
                    forwardAfterBallY = trackedBallCenter.y;
                    forwardAfterRadius = nearBallConfirmedRadius;
                    forwardDeltaBallY =
                        forwardAfterBallY - forwardBeforeBallY;
                    forwardDeltaRadius =
                        forwardAfterRadius - forwardBeforeRadius;
                    forwardAfterHeadingError = normalizeAngle(
                        shotTargetImuYaw -
                            static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    forwardReacquireSucceeded = true;
                    forwardSafetyStop =
                        std::abs(forwardAfterHeadingError) >
                            coarseYawAcceptDegrees;
                    if (forwardSafetyStop) {
                        approachPhase =
                            ApproachPhase::FORWARD_CALIBRATION_STOP;
                    } else {
                        // 先完成已验证的 T5/T6a，再切到低头近球
                        // 姿态。只有重捕获同一足球后才允许继续接近。
                        ballReacquireHeadYaw = 0.0;
                        ballReacquireHeadPitch =
                            kickObservationHeadPitch;
                        fineStepCommand = 0.0;
                        fineLateralCommand = 0.0;
                        fineForwardPulses = 0;
                        fineLateralPulses = 0;
                        fineYawCorrectionPulses = 0;
                        fineSafetyStop = false;
                        fineStopReason = "unknown";
                        beginFineBallReacquire();
                    }
                } else if (fineAdjustmentObservation) {
                    evaluateKickGeometry();
                } else {
                    coarseAfterBallX = trackedBallCenter.x;
                    coarsePixelShift =
                        coarseAfterBallX - coarseInitialBallX;
                    const int beforeError =
                        coarseBeforeBallX - coarseBeforeTargetX;
                    const int afterError =
                        coarseAfterBallX - preferredFootTargetX;
                    coarseDirectionMatched = coarseLateralCommand == 0.0
                        ? std::abs(afterError) <=
                            preferredFootTolerancePixels
                        : std::abs(afterError) < std::abs(beforeError);
                    coarseHeadingError = normalizeAngle(
                        shotTargetImuYaw -
                            static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    coarseSafetyStop =
                        !coarseDirectionMatched ||
                        std::abs(coarseHeadingError) >
                            coarseYawAcceptDegrees;
                    coarseAlignmentReached =
                        std::abs(afterError) <=
                            preferredFootTolerancePixels;
                    if (!coarseSafetyStop &&
                        !coarseAlignmentReached &&
                        coarseLateralPulses <
                            maxCoarseLateralPulses) {
                        coarseBeforeBallX = coarseAfterBallX;
                        coarseBeforeTargetX = preferredFootTargetX;
                        coarseLateralCommand =
                            coarseAfterBallX > preferredFootTargetX
                                ? -0.015 : 0.015;
                        ++coarseLateralPulses;
                        approachPhase =
                            ApproachPhase::COARSE_LATERAL_MOVE;
                        approachPhaseFrames = coarseMotionCommandFrames;
                    } else {
                        approachPhase =
                            ApproachPhase::COARSE_LATERAL_STOP;
                    }
                }
            } else if (reacquireMissFrames >= reacquireMissFrameLimit) {
                const bool initialNearBallObservation =
                    approachPhase == ApproachPhase::BALL_REACQUIRE;
                const bool forwardCalibrationObservation =
                    approachPhase ==
                        ApproachPhase::FORWARD_BALL_REACQUIRE;
                const bool fineAdjustmentObservation =
                    approachPhase == ApproachPhase::FINE_BALL_REACQUIRE;
                reacquireMatchFrames = 0;
                reacquireMissFrames = 0;
                nearBallGeometrySamples = 0;
                nearBallCenterXSum = 0.0;
                nearBallCenterYSum = 0.0;
                nearBallRadiusSum = 0.0;
                hasBallTrack = false;
                lostBallFrames = 0;
                settledLostBallFrames = 0;
                stableBallFrames = 0;
                liveBallFrames = 0;
                if (initialNearBallObservation &&
                    localReacquirePoseIndex + 1 <
                    localReacquirePoses.size()) {
                    // 按中间俯仰、低头和小幅左右补偿的顺序尝试。
                    ++localReacquirePoseIndex;
                    headTargetYaw =
                        localReacquirePoses[localReacquirePoseIndex].yaw;
                    headTargetPitch =
                        localReacquirePoses[localReacquirePoseIndex].pitch;
                    htask.yaw = headTargetYaw;
                    htask.pitch = headTargetPitch;
                } else if (initialNearBallObservation) {
                    // T4 的有限局部姿态均失败时安全停止。本轮不恢复
                    // 无限扫描，更不能在未确认足球时开始身体运动。
                    reacquireAfterTurn = false;
                    ballReacquireSucceeded = false;
                    ballReacquireSafetyStop = true;
                    ballReacquireHeadYaw =
                        static_cast<double>(headAngle.yaw);
                    ballReacquireHeadPitch =
                        static_cast<double>(headAngle.pitch);
                    approachPhase = ApproachPhase::BALL_REACQUIRE_STOP;
                } else if (forwardCalibrationObservation) {
                    reacquireAfterTurn = false;
                    forwardReacquireSucceeded = false;
                    forwardAfterHeadingError = normalizeAngle(
                        shotTargetImuYaw -
                            static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    forwardSafetyStop = true;
                    approachPhase =
                        ApproachPhase::FORWARD_CALIBRATION_STOP;
                } else if (fineAdjustmentObservation) {
                    if (ballReacquireHeadPitch <
                        kickCloseHeadPitch - 3.0) {
                        // 真球贴近画面下沿时，低头后只做一次静止复测；
                        // 未确认同一完整足球前不允许移动或踢球。
                        ballReacquireHeadYaw = 0.0;
                        ballReacquireHeadPitch = kickCloseHeadPitch;
                        beginFineBallReacquire();
                    } else {
                        reacquireAfterTurn = false;
                        fineSafetyStop = true;
                        fineStopReason = "ball lost after move";
                        finalHeadingError = normalizeAngle(
                            shotTargetImuYaw -
                                static_cast<double>(imuData.yaw)
                        ) * imuYawDirectionForPositiveTurn;
                        approachPhase = ApproachPhase::KICK_STOP;
                    }
                } else {
                    reacquireAfterTurn = false;
                    coarseAfterBallX = -1;
                    coarseDirectionMatched = false;
                    coarseHeadingError = normalizeAngle(
                        shotTargetImuYaw -
                            static_cast<double>(imuData.yaw)
                    ) * imuYawDirectionForPositiveTurn;
                    coarseSafetyStop = true;
                    approachPhase = ApproachPhase::COARSE_LATERAL_STOP;
                }
            }
        }

        // T5 确认右脚窗口和航向后才继续。常规远球执行一次已标定
        // T6a；若本轮开局自然落点已经很近，跳过固定前进脉冲，
        // 避免在视觉闭环介入前把球撞走。
        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            robotUpright &&
            approachPhase == ApproachPhase::COARSE_LATERAL_STOP &&
            coarseAlignmentReached && !coarseSafetyStop &&
            !forwardCalibrationStarted) {
            forwardCalibrationStarted = true;
            forwardReacquireSucceeded = false;
            forwardSafetyStop = false;
            forwardBeforeBallX = trackedBallCenter.x;
            forwardBeforeBallY = trackedBallCenter.y;
            forwardBeforeRadius = nearBallConfirmedRadius;
            forwardAfterBallX = -1;
            forwardAfterBallY = -1;
            forwardAfterRadius = 0;
            forwardDeltaBallY = 0;
            forwardDeltaRadius = 0;
            forwardBeforeHeadingError = normalizeAngle(
                shotTargetImuYaw - static_cast<double>(imuData.yaw)
            ) * imuYawDirectionForPositiveTurn;
            forwardAfterHeadingError = forwardBeforeHeadingError;
            forwardYawCorrectionPulses = 0;
            if (nearBallConfirmedRadius >= kickBallRadiusMin) {
                ballReacquireHeadYaw = 0.0;
                ballReacquireHeadPitch = kickObservationHeadPitch;
                fineStepCommand = 0.0;
                fineLateralCommand = 0.0;
                fineForwardPulses = 0;
                fineLateralPulses = 0;
                fineYawCorrectionPulses = 0;
                fineSafetyStop = false;
                fineStopReason = "unknown";
                beginFineBallReacquire();
            } else {
                approachPhase = ApproachPhase::FORWARD_CALIBRATION_MOVE;
                approachPhaseFrames = forwardCalibrationCommandFrames;
            }
        }

        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            !image.empty() &&
            (initialGeometryTurnComplete || initialTurnSafetyStop)) {
            cv::putText(
                image,
                std::string(initialTurnSafetyStop ? "WARNING safety stop, " : "") +
                    "initial IMU error=" + std::to_string(latestPhysicalTurnError) +
                    " deg pulses=" + std::to_string(turnPulseNumber),
                cv::Point(20, 65),
                cv::FONT_HERSHEY_SIMPLEX,
                0.65,
                cv::Scalar(255, 0, 0),
                2
            );
            if (routeTravelStarted) {
                cv::putText(
                    image,
                    std::string("route travel=") +
                        (routeTravelComplete ? "complete" : "running") +
                        " nominal=" + std::to_string(nominalLongWalkDistance) +
                        "m frames=" + std::to_string(longWalkProcessedFrames),
                    cv::Point(20, 95),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.65,
                    cv::Scalar(255, 0, 0),
                    2
                );
                cv::putText(
                    image,
                    "walk heading filtered=" +
                    std::to_string(filteredWalkHeadingError) +
                    " final=" + std::to_string(finalWalkHeadingError),
                    cv::Point(20, 125),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.65,
                    cv::Scalar(255, 0, 0),
                    2
                );
                if (ballFacingTurnComplete) {
                    const double attackTurnFromInit = normalizeAngle(
                        static_cast<double>(imuData.yaw) - initYaw
                    ) * imuYawDirectionForPositiveTurn;
                    cv::putText(
                        image,
                        "shot-line err=" +
                            std::to_string(ballFacingPhysicalError) +
                            " rel=" + std::to_string(attackTurnFromInit) +
                            " target=" +
                            std::to_string(nominalAttackTurnFromStart),
                        cv::Point(20, 155),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.55,
                        cv::Scalar(255, 0, 0),
                        2
                    );
                }
                if (approachPhase == ApproachPhase::GOAL_VIEW_TEST) {
                    const int safeLocationSamples =
                        std::max(1, goalViewLocationSamples);
                    const int initialTurnClockSeconds =
                        playStartRemainTime >= 0 &&
                            longWalkStartRemainTime >= 0
                            ? playStartRemainTime - longWalkStartRemainTime
                            : 0;
                    const int routeClockSeconds =
                        longWalkStartRemainTime >= 0 &&
                            routeCompleteRemainTime >= 0
                            ? longWalkStartRemainTime -
                                routeCompleteRemainTime
                            : 0;
                    const int shotAlignClockSeconds =
                        routeCompleteRemainTime >= 0
                            ? routeCompleteRemainTime - gameData.remain_time
                            : 0;
                    const int totalClockSeconds =
                        playStartRemainTime >= 0
                            ? playStartRemainTime - gameData.remain_time
                            : 0;
                    cv::putText(
                        image,
                        "head actual yaw=" +
                            std::to_string(static_cast<double>(headAngle.yaw)) +
                            " pitch=" +
                            std::to_string(static_cast<double>(headAngle.pitch)),
                        cv::Point(20, 185),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.55,
                        cv::Scalar(255, 0, 0),
                        2
                    );
                    cv::putText(
                        image,
                        "loc avg x=" +
                            std::to_string(
                                goalViewLocationXSum / safeLocationSamples
                            ) +
                            " z=" +
                            std::to_string(
                                goalViewLocationZSum / safeLocationSamples
                            ) +
                            " n=" + std::to_string(goalViewLocationSamples),
                        cv::Point(20, 215),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.55,
                        cv::Scalar(255, 0, 0),
                        2
                    );
                    cv::putText(
                        image,
                        "clock init=" +
                            std::to_string(initialTurnClockSeconds) +
                            " route=" + std::to_string(routeClockSeconds) +
                            " shot=" +
                            std::to_string(shotAlignClockSeconds) +
                            " total=" + std::to_string(totalClockSeconds) +
                            " remain=" +
                            std::to_string(gameData.remain_time),
                        cv::Point(20, 245),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.50,
                        cv::Scalar(255, 0, 0),
                        2
                    );
                    if (hasFrozenShotGoal || hasGoalTrack) {
                        const cv::Rect shotDisplayGoalRect =
                            hasFrozenShotGoal
                                ? frozenShotGoalRect
                                : trackedGoalRect;
                        const bool goalConfirmed =
                            hasFrozenShotGoal ||
                            goalStableFrames >= goalConfirmFrames;
                        cv::rectangle(
                            image,
                            shotDisplayGoalRect,
                            goalConfirmed
                                ? cv::Scalar(0, 255, 0)
                                : cv::Scalar(255, 165, 0),
                            3
                        );
                        if (hasFrozenShotGoal) {
                            const int candidateY = shotDisplayGoalRect.y +
                                shotDisplayGoalRect.height * 3 / 4;
                            const cv::Point leftShotCandidate(
                                shotDisplayGoalRect.x + cvRound(
                                    shotDisplayGoalRect.width *
                                        leftShotTargetU
                                ),
                                candidateY
                            );
                            const cv::Point rightShotCandidate(
                                shotDisplayGoalRect.x + cvRound(
                                    shotDisplayGoalRect.width *
                                        rightShotTargetU
                                ),
                                candidateY
                            );
                            const cv::Scalar leftCandidateColor =
                                lockedShotSide < 0
                                    ? cv::Scalar(0, 255, 0)
                                    : pendingShotSide < 0
                                        ? cv::Scalar(255, 165, 0)
                                        : cv::Scalar(255, 255, 0);
                            const cv::Scalar rightCandidateColor =
                                lockedShotSide > 0
                                    ? cv::Scalar(0, 255, 0)
                                    : pendingShotSide > 0
                                        ? cv::Scalar(255, 165, 0)
                                        : cv::Scalar(255, 255, 0);
                            cv::circle(
                                image,
                                leftShotCandidate,
                                7,
                                leftCandidateColor,
                                -1
                            );
                            cv::circle(
                                image,
                                rightShotCandidate,
                                7,
                                rightCandidateColor,
                                -1
                            );
                        }
                    }
                    if (hasKeeperTrack) {
                        cv::rectangle(
                            image,
                            trackedKeeperRect,
                            cv::Scalar(0, 0, 255),
                            3
                        );
                    } else if (t1KeeperAnchorVisible) {
                        cv::rectangle(
                            image,
                            t1KeeperAnchorRect,
                            cv::Scalar(255, 0, 255),
                            2
                        );
                    }
                    cv::putText(
                        image,
                        "T1 goal=" + std::to_string(goalStableFrames) +
                            "/" + std::to_string(goalConfirmFrames) +
                            " keeper=" +
                            std::to_string(keeperStableFrames) + "/" +
                            std::to_string(keeperConfirmFrames) +
                            " MOVE=LOCKED",
                        cv::Point(20, 275),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.55,
                        cv::Scalar(255, 0, 0),
                        2
                    );
                    cv::putText(
                        image,
                        "T1 anchor=" +
                            std::to_string(
                                t1KeeperAnchorVisible ? 1 : 0
                            ) +
                            " vlines=" +
                            std::to_string(t1VerticalLineCandidates) +
                            " method=" +
                            std::to_string(t1GoalDetectionMethod),
                        cv::Point(20, 305),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.55,
                        cv::Scalar(255, 0, 0),
                        2
                    );
                    cv::putText(
                        image,
                        "T2 keeper u=" + std::to_string(filteredKeeperU) +
                            " samples=" +
                            std::to_string(keeperMotionSamples) +
                            " frozen=" +
                            std::to_string(hasFrozenShotGoal ? 1 : 0),
                        cv::Point(20, 335),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.45,
                        cv::Scalar(255, 0, 0),
                        2
                    );
                    cv::putText(
                        image,
                        "T2 clear L=" +
                            std::to_string(leftShotClearance) +
                            " R=" + std::to_string(rightShotClearance) +
                            " lock=" +
                            std::string(
                                lockedShotSide < 0 ? "L" :
                                lockedShotSide > 0 ? "R" : "NONE"
                            ) + " wait=" +
                            std::to_string(shotDecisionWaitFrames) + "/" +
                            std::to_string(shotDecisionMaxWaitFrames),
                        cv::Point(20, 365),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.45,
                        cv::Scalar(255, 0, 0),
                        2
                    );
                }
            }
        }

        if (gameData.state == gameData.STATE_PLAY && newCameraFrame &&
            !image.empty() &&
            (approachPhase == ApproachPhase::SHOT_TARGET_TURN ||
             approachPhase == ApproachPhase::SHOT_TARGET_SETTLE ||
             approachPhase == ApproachPhase::SHOT_TARGET_STOP ||
             approachPhase == ApproachPhase::BALL_REACQUIRE ||
             approachPhase == ApproachPhase::BALL_REACQUIRE_STOP ||
             approachPhase == ApproachPhase::COARSE_LATERAL_MOVE ||
             approachPhase == ApproachPhase::COARSE_LATERAL_SETTLE ||
             approachPhase == ApproachPhase::COARSE_YAW_RECOVER ||
             approachPhase == ApproachPhase::COARSE_YAW_SETTLE ||
             approachPhase == ApproachPhase::COARSE_BALL_REACQUIRE ||
             approachPhase == ApproachPhase::COARSE_LATERAL_STOP ||
             approachPhase == ApproachPhase::FORWARD_CALIBRATION_MOVE ||
             approachPhase ==
                ApproachPhase::FORWARD_CALIBRATION_SETTLE ||
             approachPhase == ApproachPhase::FORWARD_YAW_RECOVER ||
             approachPhase == ApproachPhase::FORWARD_YAW_SETTLE ||
             approachPhase == ApproachPhase::FORWARD_BALL_REACQUIRE ||
             approachPhase == ApproachPhase::FORWARD_CALIBRATION_STOP ||
             approachPhase == ApproachPhase::FINE_MOVE ||
             approachPhase == ApproachPhase::FINE_SETTLE ||
             approachPhase == ApproachPhase::FINE_YAW_RECOVER ||
             approachPhase == ApproachPhase::FINE_YAW_SETTLE ||
             approachPhase == ApproachPhase::FINE_BALL_REACQUIRE ||
             approachPhase == ApproachPhase::KICK_REQUEST ||
             approachPhase == ApproachPhase::KICK_WAIT ||
             approachPhase == ApproachPhase::KICK_STOP)) {
            const int imageCenterX = image.cols / 2;
            cv::line(
                image,
                cv::Point(imageCenterX, 0),
                cv::Point(imageCenterX, image.rows - 1),
                cv::Scalar(0, 255, 255),
                2
            );
            cv::circle(
                image,
                cv::Point(imageCenterX, image.rows / 2),
                7,
                cv::Scalar(0, 255, 255),
                -1
            );
            if (preferredFootTargetX >= 0 &&
                preferredFootTargetX < image.cols) {
                cv::line(
                    image,
                    cv::Point(preferredFootTargetX, 0),
                    cv::Point(preferredFootTargetX, image.rows - 1),
                    cv::Scalar(255, 0, 255),
                    2
                );
                const int leftWindowX = std::max(
                    0,
                    preferredFootTargetX - preferredFootTolerancePixels
                );
                const int rightWindowX = std::min(
                    image.cols - 1,
                    preferredFootTargetX + preferredFootTolerancePixels
                );
                cv::line(
                    image,
                    cv::Point(leftWindowX, 0),
                    cv::Point(leftWindowX, image.rows - 1),
                    cv::Scalar(255, 0, 255),
                    1
                );
                cv::line(
                    image,
                    cv::Point(rightWindowX, 0),
                    cv::Point(rightWindowX, image.rows - 1),
                    cv::Scalar(255, 0, 255),
                    1
                );
            }
            cv::putText(
                image,
                "T3 side=" + std::string(
                    lockedShotSide < 0 ? "IMAGE-LEFT" : "IMAGE-RIGHT"
                ) + " visual=" +
                    std::to_string(shotTargetVisualAngleDegrees) +
                    " aim=" + std::to_string(shotTargetAimAngleDegrees) +
                    " targetYaw=" + std::to_string(shotTargetImuYaw),
                cv::Point(20, 185),
                cv::FONT_HERSHEY_SIMPLEX,
                0.50,
                cv::Scalar(255, 0, 0),
                2
            );
            cv::putText(
                image,
                "T3 yawError=" +
                    std::to_string(shotTargetPhysicalError) +
                    " deg corrections=" +
                    std::to_string(shotTargetCorrectionPulses) + "/" +
                    std::to_string(maxShotTargetCorrectionPulses),
                cv::Point(20, 215),
                cv::FONT_HERSHEY_SIMPLEX,
                0.50,
                cv::Scalar(255, 0, 0),
                2
            );
        }

        // 长走时将 640x480 调试图限制为每 5 帧发布一次，
        // 减少 DDS 图像拷贝和 rqt 重绘开销。观察和平视验证阶段
        // 仍逐帧发布，不影响视觉确认。
        const bool publishResultThisFrame =
            approachPhase == ApproachPhase::OBSERVE ||
            approachPhase == ApproachPhase::GOAL_VIEW_TEST ||
            approachPhase == ApproachPhase::SHOT_TARGET_STOP ||
            approachPhase == ApproachPhase::BALL_REACQUIRE_STOP ||
            approachPhase == ApproachPhase::COARSE_BALL_REACQUIRE ||
            approachPhase == ApproachPhase::COARSE_LATERAL_STOP ||
            approachPhase == ApproachPhase::FORWARD_BALL_REACQUIRE ||
            approachPhase == ApproachPhase::FORWARD_CALIBRATION_STOP ||
            approachPhase == ApproachPhase::FINE_BALL_REACQUIRE ||
            approachPhase == ApproachPhase::KICK_REQUEST ||
            approachPhase == ApproachPhase::KICK_WAIT ||
            approachPhase == ApproachPhase::KICK_STOP ||
            imageFrameSequence % 5ULL == 0ULL;
        if (newCameraFrame && !image.empty() && publishResultThisFrame) {
            lastResultImage = image.clone();
            resImgPublisher->Publish(lastResultImage);
        }

        bodyTaskNode->Publish(btask);
        headTaskNode->Publish(htask);

        loop_rate.sleep();
        // ----------------- 可以修改的部分 end--------------------
    }
    rclcpp::shutdown();
    return 0;
}
