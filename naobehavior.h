#ifndef NAOBEHAVIOR_H
#define NAOBEHAVIOR_H

#include "rl/RLAgent.h"
#include "behavior.h"
#include "../headers/headers.h"
#include "../parser/parser.h"
#include "../worldmodel/worldmodel.h"
#include "../bodymodel/bodymodel.h"
#include "../particlefilter/PFLocalization.h"
#include "../skills/skill.h"
#include "../rl_train/my_includes.h"
#include "../rl_train/memorise.h"

#include "../fpkick/FPKickMapping.h"


// For UT Walk
#include <MotionCore.h>
#include <memory/Memory.h>
#include <memory/FrameInfoBlock.h>
#include <memory/SensorBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/SimEffectorBlock.h>
#include <memory/WalkRequestBlock.h>

#include <queue>
#include <vector>
#include <memory>

#include "json.hpp"
using json = nlohmann::json;

using namespace std;


// TODO: Temporary home. Not sure if this this the best place to put this.
struct WalkVelocity
{
    WalkRequestBlock::ParamSet paramSet;

    double x;   // X direction velocity (unspecified unit)
    double y;   // Y direction velocity
    double rot;   // Rotational velocity about the Z-axis

    WalkVelocity() : paramSet(WalkRequestBlock::PARAMS_DEFAULT), x(0), y(0), rot(0) {}

    WalkVelocity(const double& velX, const double& velY, const double& velRot) :
        paramSet(WalkRequestBlock::PARAMS_DEFAULT), x(velX), y(velY), rot(velRot) {}

    WalkVelocity(WalkRequestBlock::ParamSet param, const double& velX, const double& velY, const double& velRot) :
        paramSet(param), x(velX), y(velY), rot(velRot) {}

    friend std::ostream& operator<<(std::ostream &out, const WalkVelocity& v)
    {
        out << "Parameter Set: " << v.paramSet << " T: (" << v.x << ", " << v.y << ") |R: " << v.rot;
        return out;
    }
};

class NaoBehavior : public Behavior {
    friend class KickClassifier;
protected:

    double currentFallStateStartTime;

    // TODO: eliminate these and use a better solution
    string classname;

    map< SkillType, boost::shared_ptr<Skill> > skills;
    const map<string, string>& namedParams;
    string rsg;

    std::string agentTeamName;
    int agentUNum;

    Parser *parser;
    WorldModel *worldModel;
    BodyModel *bodyModel;
    PFLocalization* particleFilter;

    // For UT Walk
    MotionCore* core;
    Memory *memory_;
    FrameInfoBlock* frame_info_;
    FrameInfoBlock* vision_frame_info_;
    SensorBlock* raw_sensors_;
    JointBlock* raw_joint_angles_;
    JointCommandBlock* raw_joint_commands_;
    JointBlock* processed_joint_angles_;
    JointCommandBlock* processed_joint_commands_;
    SimEffectorBlock* sim_effectors_;
    WalkVelocity velocity;
    
    // latch for kick target pass thing
    int kickLatchCount = 0;
    int MAX_KICK_LATCH_COUNT = 150;
    bool hasKickLatched = false;
    VecPosition kickLatchTarget;


    // For UT Walk
    void calculateAngles();
    void preProcessJoints();
    void postProcessJoints();

    double hoverTime;
    bool mInit;
    bool initBeamed;
    double beamTime;


    bool initialized;

    SkillType skill;
    int skillState;

    int fallState;
    bool fallenLeft, fallenRight, fallenDown, fallenUp;
    double fallTimeStamp;
    double fallTimeWait;

    VecPosition kickDirection;
    int kickType;
    VecPosition kickTarget;
    double kickVerticalAngle;

    double lastGetupRecoveryTime;

    SkillType currentKick;
    int currentKickType;

    VecPosition me;
    VecPosition myXDirection, myYDirection, myZDirection;

    VecPosition ball;

    json j;

    //USE_PASS
    bool usePass = false;

    //USE_GoalieCatch
    bool useGoalieCatch = false;

    //VIS
    bool visualise = false;

    double oldValueRHipYawPitch = 0.0;
    double currStateOldValueRHipYawPitch = 0.0;

   
    double FAT_SHOOTING_RANGE=10;
    double SHOOTING_RANGE=6.5;
    double FAT_SAFE_PASS_DISTANCE = 9;
    double SAFE_PASS_DISTANCE = 6;


    //LOG
    bool log = false;

    //SCORE
    int scoreMe;
    int scoreOpp;

    FPKickMapping FPKickMap;

    string monMsg;

    bool fParsedVision;



    RLAgent get_an_rl_model(std::string name);
    RLAgent get_rl_agent_from_name(std::string name);
    std::vector<RLAgent> all_rl_agents_indexed_by_int;
    std::map<std::string, int> all_rl_agents_index_map;

    // RL
    RLAgent myRLAgent;

    // RLAgent MY_RL_AGENT_NORMAL;
    // RLAgent MY_RL_AGENT_LONG  ;
    bool shouldUseMemorisedRLKicks = false; //true; //false;
    bool isBusyRLKick = false;
    bool shouldIgnoreVision = false;
    std::string rlKickModelToUse = "normal";
    bool hasStartedRLKick = false;
    bool shouldRLKickPID = false;
    bool shouldCorrectRLTarget = false;
    std::string rl_message;
    int mycount = 0;
    int mycount_2 = 0;
    int my_global_count = 0;
    bool has_warmed_up = false;
    int my_wait_count = 0;
    bool should_wait = false;
    std::map<std::string, double> current_state;
    
    std::vector<double> old_state_for_pid;//(0, 22);
    void resetRLAgent();

    string composeAction();


    
    virtual void resetSkills();
    void resetScales();
    void refresh();
    void act();

    // ----------------------------------------------------
    // ---------  THESE FUNCTIONS ARE
    // ---------  TO BE OVERRIDEN BY AGENTS...
    virtual SkillType selectSkill();
    virtual void beam( double& beamX, double& beamY, double& beamAngle );
    virtual void updateFitness() {}

    // ----------------------------------------------------


    bool checkingFall();

    /**
     * Trims the value to within [min, max].
     *
     * value - The value to trim.
     * min   - The minimum that the value can be.
     * max   - The maximum that the value can be.
     */
    double trim(const double& value, const double& min, const double&max);

    /**
     * Returns the skill that will best approximate the desired
     * walk direction and rotation using the currently implemented
     * walk. This function was designed for use by selectSkill().
     * It allows for a more general implementation of selectSkill()
     * that does not depend (as much) on the currently implemented
     * walk. This function delivers the faGoodstest possible speed, so it
     * is not appropriate for alignment/fine-tuning.
     *
     * For the purpose of this implementation, rotation = 0 is not
     * the same as 360 is not the same as -360. 0 means no rotation,
     * 360 means rotate to the left, while -360 means rotate to the
     * right.
     *
     * direction - The angle to walk in degrees relative to the
     *             direction the robot is facing.
     * rotation  - The angle in degrees to turn the robot.
     * speed     - The percentage of maximum walk speed to use. Should
     *             be a value between 0 and 1. Default 1. This argument
     *             does not affect turn speed.
     * fAllowOver180Turn - allow for turns greater than abs(180) instead
     *                     of mapping them to their complement
     *
     */
    SkillType getWalk(const double& direction, const double& rotation, double speed = 1.0, bool fAllowOver180Turn=false);
    SkillType getWalk(WalkRequestBlock::ParamSet paramSet, const double& direction, double rotation, double speed, bool fAllowOver180Turn=false);

    /**
     * Returns the skill that's needed to get to the target location facing the target
     * rotation. All targets are offsets relative to the robot's current location and
     * orientation. Note that something like (globalTarget - me) is NOT the correct local
     * target. g2l(globalTarget) should be used instead.
     */
    SkillType goToTargetRelative(const VecPosition& targetLoc, const double& targetRot, double speed=1, bool fAllowOver180Turn=false, WalkRequestBlock::ParamSet paramSet=WalkRequestBlock::PARAMS_DEFAULT);

    SkillType goToTarget(const VecPosition &target);

    VecPosition collisionAvoidance(bool avoidTeammate, bool avoidOpponent, bool avoidBall, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, bool fKeepDistance=true);
    VecPosition collisionAvoidanceCorrection(VecPosition start, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle);
    VecPosition collisionAvoidanceApproach(double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle);
    VecPosition collisionAvoidanceApproach(VecPosition start, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle);
    VecPosition navigateAroundBall(VecPosition target, double PROXIMITY_THRESH, double COLLISION_THRESH );

    void resetKickState();

    double computeKickCost(VecPosition target, SkillType kickType);
    SkillType kickBall(const int kickTypeToUse, const VecPosition &target, const double kickVerticalAngle=0.0);
    SkillType kickBallFat(kickOption kickTypeToUse, const VecPosition &target, VecPosition myDesiredWalkPosition);
    SkillType kickBallAtPresetTarget();

    void getTargetDistanceAndAngle(const VecPosition &target, double &distance, double &angle);

    SkillType kickBallAtTargetSimplePositioning( const VecPosition &targetToKickAt, SkillType kick_skill, int kickType);

    /**
     * Returns the maximum direction at which we can walk and still maintain maximum forward speed.
     * In other words, returns the angle theta such that if we walk in any direction in [-theta, theta],
     * our forward translation will be the maximum that the walk engine will allow.
     */
    double getLimitingAngleForward();

    bool beamablePlayMode();
    bool improperPlayMode();
    bool improperPlayMode(int pm);
    bool kickPlayMode();
    bool kickPlayMode(int pm, bool eitherTeam=false);
    bool isIndirectKick();
    bool isIndirectKick(int pm);

    void readSkillsFromFile( const std::string& filename);

    bool isRightSkill( SkillType skill );
    bool isLeftSkill( SkillType skill );

    double getParameter(const std::string& name);
    double getStdNameParameter(const SkillType kick_skill, const std::string& parameter);
    void getSkillsForKickType(int kickType, SkillType skillsForType[]);

    SkillType demoKickingCircle();

    //Our Functions

    SkillType goalKeeperBehaviour();
    SkillType centreBackBehaviour();
    SkillType onBallBehaviour();
    SkillType WitsGoToFormationBehaviour();

    SkillType WitsDecisionTreeBehaviour();
    SkillType WitsDecisionTreeBehaviour2023();
    SkillType WitsFATDecisionTreeBehaviour();
    SkillType WitsStandWithGoalieCatch();


    int playerClosestToBall();
    int roleAssignment(int closestToBall, std::vector<pair<double, double>> starts, std::vector<pair<double, double>> targets);
    void getPositions(int closestToBall, std::vector<pair<double, double>> &starts, std::vector<pair<double, double>> &targets);

    vector<VecPosition> GenerateImportantPositions(int _playMode, int _side, int _playerNumber, int regionX, int regionY);
    vector<VecPosition> GenerateImportantPositionsFAT(int _playMode, int _side, int _playerNumber, int regionX, int regionY);
    vector<VecPosition> GenerateImportantPositionsOld(int _playMode, int _side, int _playerNumber, int regionX, int regionY);

    SkillType FaceBall(VecPosition mypos);
    SkillType FaceTarget(VecPosition mypos, VecPosition target);
    SkillType SmartGoToTarget(VecPosition target,double proximity);
    SkillType SmartGoToTargetWithPrecision(VecPosition target, double proximity,double localprecision);
    SkillType SmartGoToTargetWithPrecisionAndFaceSpecificLocation(VecPosition target, double proximity,double localprecision, VecPosition _mypos, VecPosition FaceLocation);

    SkillType DetermineAppropriateKick(VecPosition _mypos, vector<pair<double,int > > OppDistToBall, VecPosition target);
    SkillType DetermineAppropriateRLKick(VecPosition _mypos, vector<pair<double,int > > OppDistToBall, VecPosition target);

    SkillType doSetPlays(int _playMode, int _playerNumber, vector<pair<double,int > > TeamDistToBall, VecPosition mydesiredposition, VecPosition _myPos, vector<int> PointPreferences, vector<VecPosition> ImportantPositions );


    bool isTargetDirectlyBehindMe(VecPosition _mypos, VecPosition target);
    bool isPossesion(vector<pair<double,int > > TeamDistToBall, vector<pair<double,int > > OppDistToBall);
    bool isSafeToKick(vector<pair<double,int > > OppDistToBall);
    bool isClosestPlayertoBall(int _playerNumber, vector<pair<double,int > > TeamDistToBall, vector<pair<double,int > > OppDistToBall);
    bool isShootingRange(VecPosition _player, double range);
    bool BallinMyGoalMouth();
    bool BallinOppGoalMouth();
    bool atDestination(VecPosition mypos, VecPosition target);
    bool isInNeighbourhood(VecPosition obj, VecPosition target, double threshold);
    bool isAnyTeammateInNeighbourhood(int pNum, VecPosition target, double threshold);
    bool isBetweenTargetAndBall(VecPosition target, VecPosition mypos, double threshold);
    bool isClosestTeam(int _player, vector<pair<double,int > > TeamDistToBall);
    bool DangerClose(vector<pair<double,int > > OppDistToBall);
    bool isOffensiveStrat(int pNum, vector<pair<double,int > > TeamDistToBall, vector<pair<double,int > > OppDistToBall);
    bool AmITheNthClosestTeammateToBall(vector<pair<double,int > > TeamDistToBall, int _playerNumber, int position); 
    bool isThereObstructionBetweenTargetAndBall(VecPosition target, double threshold=1, double width=0.25);
    
    bool isOpponentWithinRegion(VecPosition playerPos, double threshold);

    void DetermineUsePass(vector<pair<double,int > > OppDistToBall);
    void DetermineUseGoalieCatchBall(int _playerNumber);
    
    int getXFieldRegion(VecPosition mypos);
    int getYFieldRegion(VecPosition objpos);

    vector<pair<double,int > > GenerateTeamToTargetDistanceVector(int _playernumber, VecPosition target);
    vector<pair<double,int > > GenerateOppToTargetDistanceVector(VecPosition target);

    VecPosition getClosestTeammatePos(int _playerNumber,vector<pair<double,int > > TeamDistToBall);
    VecPosition ValidateFieldPos(VecPosition target);
    VecPosition GetDesiredTargetAdvanced(int pNum, VecPosition mypos, int regionX, int regionY, vector<pair<double,int > > TeamDistToBall);
    VecPosition DetermineValidPassRecipient(VecPosition currTarget, VecPosition mypos, int _playerNumber, vector<pair<double,int > > TeamDistToBall, bool passbackallowed);
    VecPosition DetermineShootAtGoalPosition(VecPosition _myPos, int _playerNumber, vector<pair<double,int > > TeamDistToBall);
    VecPosition DetermineShootAtGoalPositionOld(VecPosition _myPos, int _playerNumber, vector<pair<double,int > > TeamDistToBall);

    VecPosition ExtendShotInDirection(VecPosition _myPos, VecPosition _target, double distance);
    VecPosition OptimalPassPosition(int _playerNumber, VecPosition initialtarget, vector<int> PointPreferences,vector<VecPosition> ImportantPositions);

    VecPosition DetermineValidPassPositionPlayerCentric(VecPosition _myPos, vector<pair<double,int > > TeamDistToOppGoal, vector<pair<double,int > > TeamDistToBall, VecPosition playerPos, int kickingDistanceThreshold, int _playerNumber, bool passBackAllowed);
    VecPosition DetermineValidPassPositionPlayerCentricFAT(vector<pair<double,int > > TeamDistToOppGoal, vector<pair<double,int > > TeamDistToBall, VecPosition playerPos, int kickingDistanceThreshold, int _playerNumber, bool passBackAllowed);

    VecPosition DetermineGoodKickToLocation(int _playernumber , VecPosition _myPos, double minKickDistThreshold=6);


    int DetermineValidPassIDPlayerCentric(vector<pair<double,int > > TeamDistToOppGoal,vector<pair<double,int > > TeamDistToBall, VecPosition playerPos, int kickingDistanceThreshold);
    bool isPassReciever(vector<pair<double,int > > TeamDistToBall);

    double MapValueInRangeTo(double input, double input_start, double input_end, double output_start, double output_end);

    vector<int> stableMarriage(vector<vector<pair<double,int > > > PreferenceToPointArray);
    bool wPrefersM1OverM(int prefer[2*NUM_AGENTS][NUM_AGENTS], int w, int m, int m1);
    vector<pair<double,int > > sortArr(vector<pair<double, double> > arr,int n, pair<double, double> p);
    vector<pair<double,int > > GeneratePreferenceArrayForAnAgent(vector<VecPosition> ImportantPositions, VecPosition playerpos);
    vector<vector<pair<double,int > > > GeneratePreferenceArrayForTeam(int _playerNumber, vector<VecPosition> ImportantPositions);
    
    
    std::vector<double> get_joints();
    void print_state();
    double norm(std::vector<double> A);
    std::map<std::string, double> getCurrentStateDictionaryRL(bool updateTimeSteps=true);
    void warmup_rl_models();
    RLAgent setUpRLAgent();
    SkillType doRLKick(const VecPosition &target, std::string RL_MODEL="normal");
    #ifdef IS_RL_TRAIN
    std::string get_action_from_rl_training(const std::string& message);
    #endif






// ------------- Mamadou ----------------------
 int player_possess_ball = -1;
 bool player_kick_ball = false;
 float prev_dist = -1;


// Determine how long a team mate will wait to receive
// the ball
double m_timer_recei_ball = 0;
bool m_has_made_pass = false;
bool m_teammate_has_recei_ball = false;
double m_timer_to_kick = 0;
bool m_timer_kick_on = false;

VecPosition m_initial_ball_pos = VecPosition (-1,-1,-1);

VecPosition m_prev_ball_pos = VecPosition(-1,-1,-1);

bool m_is_passer;
bool m_is_receiver;

bool m_initialized =  false;


bool HasMadeAPass(VecPosition initial_ball_pos);
void InitPlayer();
void OnPassingBall(int pNum,int teamNum,VecPosition myPos,VecPosition teamPos);
void OnReceivingBall(int pNum,int teamNum,VecPosition myPos,VecPosition teamPos);
void OnWaitingTeamToReceive(int pNum,int teamNum,VecPosition myPos,VecPosition teamPos);


bool DidBallStop();



// ----------------------------------------------------



public:

    NaoBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_);
    virtual ~NaoBehavior();

    virtual std::string Init();
    virtual std::string Think(const std::string& message);

    double roundToSig(double value, int significantFigures);

    void JSONLOG_DataToLogAlways();
    void JSONLOG_WriteToFile();
    void JSONLOG_StrategyDataToLog(string Actiontype,VecPosition location);

    void setMonMessage(const std::string& msg);
    string getMonMessage();

    inline MotionCore* getCore() {
        return core;
    }


    public:

    // Bad RL Things
    const std::map<int, std::vector<std::string>> RL_MEMORISED_KICKS = {
        {0, RL_MEMORISED_KICK_TYPE_ZERO},
        {1, RL_MEMORISED_KICK_TYPE_ONE},
        {2, RL_MEMORISED_KICK_TYPE_TWO},
        {3, RL_MEMORISED_KICK_TYPE_THREE}
    };
    virtual bool isAPenaltyKicker(){return false;}
};

#endif // NAOBEHAVIOR_H
