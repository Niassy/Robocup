#include "naobehavior.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <cctype>
#include <chrono>
#include <exception>

#include "../skills/skillparser.h"
#include "../rvdraw/rvdraw.h"
#include <assert.h>

// For UT Walk
#include <common/InterfaceInfo.h>
#include <motion/MotionModule.h>

#include "../rl_train/my_includes.h"
using namespace std::chrono;

extern int agentBodyType;

RLAgent NaoBehavior::get_an_rl_model(std::string name){
    return RLAgent::makeRLAgent("./behaviors/rl/rl_models/kick/" + name + "/export.env", "./behaviors/rl/rl_models/kick/" + name + "/TORCH_CPP_MODEL.pt");   
}

/*
 * namedParams_ are a mapping between parameters and their values
 */
NaoBehavior::
NaoBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_) :
    namedParams( namedParams_ ),
    rsg( rsg_ ), myRLAgent(setUpRLAgent()),
    old_state_for_pid(std::vector<double>(22, 0)) //22 joints
{
    //cout << "Constructing of Nao Behavior" << endl;

    srand ((unsigned)time(NULL) );
    srand48((unsigned)time(NULL));

    classname = "NaoBehavior"; //TODO: eliminate it...

    mInit = false;
    initBeamed = false;

    agentTeamName = teamName;
    agentUNum = uNum;

    scoreMe = 0;
    scoreOpp = 0;

    worldModel = new WorldModel();
    bodyModel = new BodyModel(worldModel);

    memory_ = new Memory(false,true);

    memory_->getOrAddBlockByName(frame_info_,"frame_info");
    memory_->getOrAddBlockByName(vision_frame_info_,"vision_frame_info");
    frame_info_->source = MEMORY_SIM; // set to simulaor
    vision_frame_info_->source = MEMORY_SIM;

    memory_->getOrAddBlockByName(raw_sensors_,"raw_sensors");
    memory_->getOrAddBlockByName(raw_joint_angles_,"raw_joint_angles");
    memory_->getOrAddBlockByName(processed_joint_angles_,"processed_joint_angles");
    memory_->getOrAddBlockByName(raw_joint_commands_,"raw_joint_commands");
    memory_->getOrAddBlockByName(processed_joint_commands_,"processed_joint_commands");
    memory_->getOrAddBlockByName(sim_effectors_,"sim_effectors");

    core = new MotionCore(CORE_SIM, true, *memory_);
    fParsedVision = false;
    particleFilter = new PFLocalization( worldModel, bodyModel, core);

    parser = new Parser(worldModel, bodyModel, teamName, particleFilter,
                        vision_frame_info_,
                        frame_info_,
                        raw_joint_angles_,
                        raw_sensors_ );



    initBeamed = false;
    initialized = false;
    beamTime = -1;
    hoverTime = 2.25;

    fallState = 0;
    fallenLeft = false;
    fallenRight = false;
    fallenDown = false;
    fallenUp = false;
    fallTimeStamp = -1;
    fallTimeWait = -1;

    lastGetupRecoveryTime = -1.0;

    monMsg = "";

    // TODO: Treat paths more correctly? (system independent way)
    try {
        readSkillsFromFile( "./skills/stand.skl" );
        readSkillsFromFile( "./skills/kick.skl" );

        // ik skills
        readSkillsFromFile( "./skills/kick_ik_0.skl" );
        // end ik skills

        // golie skills
        readSkillsFromFile( "./skills/lateral_dive.skl" );
        readSkillsFromFile( "./skills/central_split_dive.skl" );
        // end golie skills


    }
    catch( std::string& what ) {
        cerr << "Exception caught: " << what << endl;
        exit(1);
    }
    catch (std::exception& e)
    {
        cerr << e.what() << endl;
        exit(1);
    }

    // initialize just so reset Skill doesnt segfault
    skill = SKILL_STAND;
    resetSkills();
    resetKickState();

    // Uncomment this to use ground truth data for localization
    //worldModel->setUseGroundTruthDataForLocalization(true);
    // setUpRLAgent();
}

NaoBehavior::~NaoBehavior() {

    delete parser;
    delete worldModel;
    delete bodyModel;
    delete particleFilter;
    delete core;
}

string NaoBehavior::Init() {
    cout << "Loading rsg: " << "(scene " << rsg << ")" << endl;
    return "(scene " + rsg + ")";
}

void NaoBehavior::JSONLOG_WriteToFile(){

    if (log){
        int _playerNumber = worldModel->getUNum();
        std::string SpNum = std::to_string(_playerNumber);
        ofstream fw("./BehaviourLogs/"+SpNum+".json", std::ios::app);
        if (fw.is_open())
        {
            fw << j <<"\n";
        }
    }
}


double NaoBehavior::roundToSig(double value, int significantFigures) {
    if (value == 0.0) {
        return 0.0;
    }

    double scale = std::pow(10, significantFigures - std::ceil(std::log10(std::abs(value))));
    return std::round(value * scale) / scale;
}

#ifdef IS_RL_TRAIN
std::string NaoBehavior::get_action_from_rl_training(const std::string& message){
    // printf("In this function Received %s\n", message.c_str()); fflush(stdout);
    // RL state
    StateDict stateDict = getCurrentStateDictionaryRL(true);


    // for(auto it = stateDict.begin(); it != stateDict.end(); ++it)
    // {
    //     std::cout << "KEY::" << it->first << " " << it->second << "\t";
    // }
    // std::cout << "\n";


    std::vector<double> rl_state = myRLAgent.get_full_processed_state(stateDict);

    // Now convert this vector into a string
    std::stringstream ss_rl;
    ss_rl << "[";
    for (int i=0; i < rl_state.size(); ++i){
        if (i != 0) ss_rl << ", ";

        ss_rl << std::to_string(rl_state[i]);
    }
    ss_rl << "]";
    
    // get RL state. TODO
    std::string rl_state_str = ss_rl.str();
    // printf("C++ Test\n"); fflush(stdout);

    // here, send the state AND MESSAGE to python
    int tot_len = message.length() + rl_state_str.size() + 256;
    char* all_data = new char[tot_len];
    std::string divider = "<<<|||>>>";
    memset(all_data, 0, tot_len);
    int idx = 0;
    for (int i = 0; i < divider.length(); ++i){
        all_data[idx++] = divider[i];
    }

    for (int i = 0; i < message.length(); ++i){
        all_data[idx++] = message[i];
    }
    for (int i = 0; i < divider.length(); ++i){
        all_data[idx++] = divider[i];
    }
    for (int i = 0; i < rl_state_str.length(); ++i){
        all_data[idx++] = rl_state_str[i];
    }
    for (int i = 0; i < divider.length(); ++i){
        all_data[idx++] = divider[i];
    }
    // printf("C++ Waiting to Send `%s`\n", all_data); fflush(stdout);
    send(new_socket, all_data, idx, 0);
    // and receive the action. TODO
    // printf("Hey there, did send the thing\n");fflush(stdout);
    char action_data[4096];
    // printf("C++ Waiting to Receive\n");fflush(stdout);
    int numbytes = recv(new_socket, action_data, 4096, 0);
    std::string proper_action(action_data, numbytes);
    // printf("Hey there, did receive the thing %s\n", proper_action.c_str());fflush(stdout);

    int index;
    std::string thing_to_send;
    const std::string ARRAY = "ARRAY:";
    const std::string PASSTHROUGH = "PASSTHROUGH:";
    const std::string PID = "PID:";
    const std::string RESET = "RESET:";
    worldModel->getRVSender()->drawCircle(0, 0, 10, 1, 0, 0); //RED
    if (proper_action.find(ARRAY) != std::string::npos) {
        proper_action = proper_action.substr(ARRAY.length() + 1);
        proper_action = proper_action.substr(0, proper_action.length() - 1);
        std::vector<double> all_things;
        stringstream ss(proper_action);
        std::string word;
        while (!ss.eof()) {
            std::getline(ss, word, ',');
            all_things.push_back(std::atof(word.c_str()));
        }
        thing_to_send = myRLAgent.get_rl_message_from_action_space(all_things, stateDict);
        // printf("Sending message here %s || INPUT: %s\n", thing_to_send.c_str(), proper_action.c_str());
        // printf("RLE2: %lf -- %lf\n", stateDict["RHipRoll"], raw_joint_angles_->values_[9]);
        // return "(lle1 5)";
    } else if (proper_action.find(PID) != std::string::npos){
        // Here I want to do PID
        std::vector<double> current_angles = myRLAgent.action_space.get_current_angles(stateDict);
        std::vector<double> velocities;
        double sums = 0;
        for (int i=0; i < current_angles.size(); ++i){
            double target_angle = 0;
            if (myRLAgent.observation_space.do_pid_to_standard_standing_pos){
                std::string name_of_this_joint = myRLAgent.action_space.effector_names[i];
                name_of_this_joint = EFFECTORS_TO_JOINTS[name_of_this_joint];
                target_angle = myRLAgent.observation_space.RL_INIT_FOR_PID[name_of_this_joint];
            }

            velocities.push_back((target_angle - current_angles[i]) * 0.1);
            sums += current_angles[i] * current_angles[i];
        }
        thing_to_send = myRLAgent.action_space.get_message(velocities);

        // also reset when doing PID

        mycount = 0;
        myRLAgent.reset();

    } else if (proper_action.find(RESET) != std::string::npos){
        // Just reset the things.
        // printf("DOING RESET NOW\n");
        this->resetRLAgent();
        thing_to_send = "(syn)";
    } else if (proper_action.find(PASSTHROUGH) != std::string::npos){
        thing_to_send = proper_action.substr(PASSTHROUGH.length());
        // printf("Sending passthrough %s\n", thing_to_send.c_str());
    } else{
        printf("IS FAILURE\n");
        exit(EXIT_FAILURE);
    }
    
    delete[] all_data;
    return thing_to_send;
}
#endif

void NaoBehavior::JSONLOG_DataToLogAlways(){
    // create a map

    int _playerNumber = worldModel->getUNum();
    VecPosition _myPos = worldModel->getMyPosition();
    _myPos.setZ(0);

    vector<pair<double,int > > TeamDistToBall = GenerateTeamToTargetDistanceVector(_playerNumber, ball);
    vector<pair<double,int > > OppDistToBall = GenerateOppToTargetDistanceVector(ball);


    vector<VecPosition> OppPos;
    vector<VecPosition> TeamPos;

    for(int i = WO_OPPONENT1; i<WO_OPPONENT1+NUM_AGENTS;i++){ //OPP PLAYERS
        WorldObject* opponent = worldModel->getWorldObject(i);
        VecPosition temp;
        temp = opponent->pos;
        temp.setZ(0);
        OppPos.push_back(temp);
    }


    for(int i = WO_TEAMMATE1; i<WO_TEAMMATE1+NUM_AGENTS;i++){ //OUR PLAYERS
        WorldObject* teammate = worldModel->getWorldObject(i);
        VecPosition temp;

        if(i == _playerNumber){
            temp = worldModel->getMyPosition();
        }
        else{
            temp = teammate->pos;
        }
        temp.setZ(0);
        TeamPos.push_back(temp);
    }

    //std::map<std::string, int> m1 {{"key", 1}};

    // create and print a JSON from the map


    int significantFigures = 5;
    
    j["MyPosition"] = {roundToSig(_myPos.getX(),significantFigures),roundToSig(_myPos.getY(),significantFigures)};
    j["BallPosition"] = {roundToSig(ball.getX(),significantFigures), roundToSig(ball.getY(),significantFigures)};

    for (int i=0; i< TeamDistToBall.size(); i++){
         j["TeamMateDistanceToBall"][std::to_string(TeamDistToBall[i].second)] = roundToSig(TeamDistToBall[i].first,significantFigures);
    }

    for (int i=0; i< OppPos.size(); i++){
        string name = "OPP" + std::to_string(i+1);
         j["OpponentPositions"][name] = {roundToSig(OppPos[i].getX(),significantFigures), roundToSig(OppPos[i].getY(),significantFigures)};
    }

    for (int i=0; i< OppPos.size(); i++){
        string name = "TEAM" + std::to_string(i+1);
         j["TeamMatePositions"][name] = {roundToSig(TeamPos[i].getX(),significantFigures), roundToSig(TeamPos[i].getY(),significantFigures)};
    }

    j["IsFallen"] = worldModel->isFallen();
    j["CurrTime"] = worldModel->getTime();
    j["CurrGameTime"] = worldModel->getGameTime();
    j["CurrPlayMode"] = worldModel->getPlayMode();

}

RLAgent NaoBehavior::get_rl_agent_from_name(std::string name){
    if (all_rl_agents_index_map.find(name) == all_rl_agents_index_map.end()) {
        return get_rl_agent_from_name("normal");
    }
    int idx = all_rl_agents_index_map[name];
    return all_rl_agents_indexed_by_int[idx];
}


void NaoBehavior::resetRLAgent(){
    // printf("RL Resetting\n");
    hasStartedRLKick = false;
    mycount_2 = 0;
    mycount = 0;
    currStateOldValueRHipYawPitch = 0.0;
    myRLAgent.reset();
    current_state.clear();
    // MY_RL_AGENT_NORMAL.reset();
    // MY_RL_AGENT_LONG.reset();
    for (int i=0; i < all_rl_agents_indexed_by_int.size(); ++i){
        all_rl_agents_indexed_by_int[i].reset();
    }
}

void do_profile(std::chrono::time_point<std::chrono::high_resolution_clock> beg, std::string name){
    // return;
    auto end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end - beg);
    std::cout << "\tElapsed Time [" << name << "]: " << duration.count() <<" us\n";
}

string NaoBehavior::Think(const std::string& message) {
    //  std::cout << "---___---\n";
    //  cout << "(NaoBehavior) received message " << message << endl;

    
    my_global_count++;
    if (!has_warmed_up && my_global_count >= 5){
        warmup_rl_models();
        has_warmed_up = true;
    }
    fParsedVision = false;
    #ifdef MIKE_IS_PROFILING
    auto beg = high_resolution_clock::now();
    #endif
    bool parseSuccess = parser->parse(message, fParsedVision);
    #ifdef MIKE_IS_PROFILING
    do_profile(beg, (std::string)"Parse " + (fParsedVision ? (std::string)"w/ Vision" : (std::string)"No Vision") + (std::string)" with message size " + std::to_string(message.length()) );
    #endif
    if(!parseSuccess && (worldModel->getPlayMode() != PM_BEFORE_KICK_OFF)) {
   cout << "****************************************\n";
   cout << "Could not parse message: " << message << "\n";
   cout << "****************************************\n";
    }
    #ifdef MIKE_IS_PROFILING
    beg = high_resolution_clock::now();
    #endif
    
    //  cout << "\nparseSuccess: " << parseSuccess << "\n";
    //  worldModel->display();
    bodyModel->refresh();
    if (!shouldIgnoreVision){
        if(fParsedVision) {
            if (!worldModel->isFallen()) {
                parser->processVision();
            } else {
                parser->processSightings(true /*fIgnoreVision*/);
            }
        }
    }
    this->updateFitness();
    #ifdef MIKE_IS_PROFILING
    do_profile(beg, (std::string)"In this iff thingy with ignoring vision = " + (std::string)(shouldIgnoreVision ? "true" : "false"));
    beg = high_resolution_clock::now();
    #endif
    //  bodyModel->display();
    //  bodyModel->displayDerived();

    // Example usage of the roboviz drawing system and RVSender in rvdraw.cc.
    // Draw agent positions and orientations
    /*
    worldModel->getRVSender()->clearStaticDrawings();
    VecPosition pos = worldModel->getMyPosition();
    VecPosition dir = VecPosition(1,0,0);
    dir = dir.rotateAboutZ(-worldModel->getMyAngDeg());
    worldModel->getRVSender()->drawPoint(pos.getX(), pos.getY(), 10);
    worldModel->getRVSender()->drawLine(pos.getX(), pos.getY(), pos.getX()+dir.getX(), pos.getY()+dir.getY());
    */

    calculateAngles();


    if (frame_info_->start_time == -1) {
        frame_info_->start_time = frame_info_->seconds_since_start;
        vision_frame_info_->start_time = frame_info_->start_time;
    }
    frame_info_->seconds_since_start= frame_info_->seconds_since_start - frame_info_->start_time;
    
    oldValueRHipYawPitch = raw_joint_angles_->values_[RHipYawPitch];
    raw_joint_angles_->values_[RHipYawPitch] = raw_joint_angles_->values_[LHipYawPitch];

    preProcessJoints();  // Apply the correct sign to the joint angles

    postProcessJoints(); // Flip the joint angles back

    string action;

    if (!mInit) {

        mInit = true;
        stringstream ss;
        ss << "(init (unum " << agentUNum << ")(teamname " << agentTeamName << "))";
        action = ss.str();
        return action;
    }

    if (worldModel->getLastPlayMode() != worldModel->getPlayMode() &&
            (worldModel->getPlayMode() == PM_BEFORE_KICK_OFF ||
             worldModel->getPlayMode() == PM_GOAL_LEFT ||
             worldModel->getPlayMode() == PM_GOAL_RIGHT)) {
        initBeamed = false;
    }
    #ifdef MIKE_IS_PROFILING
    do_profile(beg, "after the other things");
    beg = high_resolution_clock::now();
    #endif
    // Record game score
    if (worldModel->getScoreLeft() != -1 && worldModel->getScoreRight() != -1) {
        scoreMe = worldModel->getSide() == SIDE_LEFT ? worldModel->getScoreLeft() : worldModel->getScoreRight();
        scoreOpp = worldModel->getSide() == SIDE_LEFT ? worldModel->getScoreRight() : worldModel->getScoreLeft();
    }


    if ((worldModel->getPlayMode() == PM_GOAL_LEFT || worldModel->getPlayMode() == PM_GOAL_RIGHT || worldModel->getPlayMode() == PM_BEFORE_KICK_OFF) && worldModel->getLastPlayMode() != worldModel->getPlayMode()) {
        beamTime = worldModel->getTime() + hoverTime;
    }

    else if(beamTime >= 0 && worldModel->getTime() >= beamTime) {
        //initialized = false;
        initBeamed = false;
        beamTime = -1.0;
    }


    if (worldModel->getPlayMode() != worldModel->getLastPlayMode()) {
        worldModel->setLastDifferentPlayMode(worldModel->getLastPlayMode());
    }
    worldModel->setLastPlayMode(worldModel->getPlayMode());

    if(!initialized) {
        if(!worldModel->getUNumSet() || !worldModel->getSideSet()) {
            //      cout << "UNum and side not received yet.\n";
            action = "";
            return action;
        }

        if(!initBeamed) {
            initBeamed = true;


            double beamX, beamY, beamAngle;

            // Call a virtual function
            // It could either be implemented here (real game)
            // or in the inherited classes
            // Parameters are being filled in the beam function.
            this->beam( beamX, beamY, beamAngle );
            stringstream ss;
            ss << "(beam " << beamX << " " << beamY << " " << beamAngle << ")";
            particleFilter->setForBeam(beamX, beamY, beamAngle);
            action = ss.str();
            return action;
        }
        else {
            // Not Initialized
            bodyModel->setInitialHead();
            bodyModel->setInitialArm(ARM_LEFT);
            bodyModel->setInitialArm(ARM_RIGHT);
            bodyModel->setInitialLeg(LEG_LEFT);
            bodyModel->setInitialLeg(LEG_RIGHT);
            initialized = true;
        }
    }

    if(!initBeamed) {
        initBeamed = true;

        double beamX, beamY, beamAngle;

        // Call a virtual function
        // It could either be implemented here (real game)
        // or in the inherited classes - for optimization agent.
        // Parameters are being filled in the beam function.
        this->beam( beamX, beamY, beamAngle );
        stringstream ss;
        ss << "(beam " << beamX << " " << beamY << " " << beamAngle << ")";
        particleFilter->setForBeam(beamX, beamY, beamAngle);
        action = ss.str();
    }

    frame_info_->frame_id++;
    std::string oldaction = action;
    #ifdef IS_RL_TRAIN
    // TODO get RL action from Python
    // first send message
    action = get_action_from_rl_training(message);
    worldModel->getRVSender()->refresh();
    #else    
    #ifdef MIKE_IS_PROFILING
    do_profile(beg, "Before ACT");
    #endif
#ifndef IS_RL_TRAIN
    JSONLOG_DataToLogAlways();
#endif 
    #ifdef MIKE_IS_PROFILING
    beg = high_resolution_clock::now();
    #endif
    act();
    #ifdef MIKE_IS_PROFILING
    do_profile(beg, "Act");
    beg = high_resolution_clock::now();
    #endif
#ifndef IS_RL_TRAIN
    JSONLOG_WriteToFile();
#endif
#ifdef MIKE_IS_PROFILING
    do_profile(beg, "The Json Log"); 
    beg = high_resolution_clock::now();
#endif

    worldModel->getRVSender()->refresh();
    #ifdef MIKE_IS_PROFILING
    do_profile(beg, "World Model Refresh"); 
    beg = high_resolution_clock::now();
    #endif
    action = action + composeAction();
    
    #ifdef MIKE_IS_PROFILING
    do_profile(beg, "Compose action"); 
    beg = high_resolution_clock::now();
    #endif
    #endif
    #ifdef MIKE_IS_PROFILING
    do_profile(beg, "Rest of Think");
    #endif
    return action;
}

void NaoBehavior::act() {
    refresh();


    int _playerNumber = worldModel->getUNum();
    DetermineUseGoalieCatchBall(_playerNumber);



    const double LAST_LINE_SIGHTING_THRESH = 0.1;
    if (worldModel->getTime()-worldModel->getLastLineSightingTime() > LAST_LINE_SIGHTING_THRESH) {
        worldModel->setLocalized(false);
    }


    // If the ball gets too far away, reset kick state
    if(me.getDistanceTo(ball) > 1) {
        resetKickState();
    }

    //worldModel->getRVSender()->drawPoint("me", me.getX(), me.getY(), 20);
    int pm = worldModel->getPlayMode();
    bool resetForKickoff = pm == PM_BEFORE_KICK_OFF || pm == PM_GOAL_LEFT || pm == PM_GOAL_RIGHT;

    //printf("Hello Michael in Here %d\n", mycount_2);
    // if (mycount_2 >= 60 || !isBusyRLKick){
    // if (mycount_2 >= 60 && isBusyRLKick && isKickSkill(worldModel->getLastSkill()) ){
    //     resetSkills();
    //     bodyModel->setUseOmniWalk(false);
    // }
    if(checkingFall()) {
        // if (mycount_2 >= 60 || !isBusyRLKick){
        // }
        resetSkills();
        bodyModel->setUseOmniWalk(false);
        return;
    }
    else if(resetForKickoff) {
        if (beamablePlayMode() && (worldModel->isFallen() || worldModel->getTime() <= beamTime)) {
            initBeamed = false;
        }
        resetSkills();
        skill = SKILL_STAND;
        core->move(0,0,0);
        velocity.paramSet = WalkRequestBlock::PARAMS_DEFAULT;
    }
    else {
        if(skills[skill]->done( bodyModel, worldModel) ||
                bodyModel->useOmniWalk()) {
            skills[skill]->reset();
            resetScales();
            SkillType currentSkill = selectSkill();


            if (currentSkill != SKILL_WALK_OMNI) {
                velocity.x = 0;
                velocity.y = 0;
                velocity.rot = 0;
                velocity.paramSet = WalkRequestBlock::PARAMS_DEFAULT;
            }

            bodyModel->setUseOmniWalk(true);
            switch(currentSkill) {
            case SKILL_WALK_OMNI:
                core->move(velocity.paramSet, velocity.x, velocity.y, velocity.rot);
                break;
            case SKILL_STAND:
                core->move(0,0,0);
                break;
            default:
                bodyModel->setUseOmniWalk(false);
            }

            if (bodyModel->useOmniWalk()) {
                resetSkills();
            } else {

                /*EnumParser<SkillType> enumParser;
                cout << "Skill: " << enumParser.getStringFromEnum(skill) << endl;*/

                // Transitions, coding a finite state machine...

                SkillType lastSkill = worldModel->getLastSkill();
                skill = currentSkill;


            }
        }
    }
    //  cout << "Executing: " << EnumParser<SkillType>::getStringFromEnum(skill) << endl;
    // "Did Execute: " << EnumParser<SkillType>::getStringFromEnum(worldModel->getLastSkill()) <<
    //  cerr << "Selected skill: " << SkillType2Str[skill] << " time: " << worldModel->getTime() << endl;
    //    LOG_ST(skill);
    skills[skill]->execute( bodyModel, worldModel );



    worldModel->setLastSkill(skill);
    // to be used by odometry
    if (bodyModel->useOmniWalk()) {
        worldModel->addExecutedSkill(SKILL_WALK_OMNI);
    } else {
        worldModel->addExecutedSkill( skill );
    }




    //Set the head turn behavior
    VecPosition me = worldModel->getMyPosition();
    me.setZ(0);
    ball = worldModel->getBall();
    ball.setZ(0);
    // Currently, every 2 seconds
    static double panOffset = drand48() * 4.0;
    int panState = ( static_cast<int>( worldModel->getTime()+panOffset ) ) % 4;
    double ballDistance, ballAngle;

    getTargetDistanceAndAngle(ball, ballDistance, ballAngle);
    //SkillType lastSkill = worldModel->getLastSkill();

    if (worldModel->isFallen()) {
        bodyModel->setScale(EFF_H1, 0.5);
        bodyModel->setTargetAngle(EFF_H1, 0);
    } else if (ballDistance < 1.0 && worldModel->getWorldObject(WO_BALL)->validPosition) {
        // close to the ball, focusing on the ball and turning head 30 degrees
        if( panState == 0 || panState == 2 ) {
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, ballAngle);
        } else {
            int direction = (panState == 1) ? 1 : -1;
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, ballAngle+(direction*30.0));
        }
    } else {
        // default behavior
        if( panState == 0 || panState == 2 ) {
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, 0);
        } else {
            int direction = (panState == 1) ? 1 : -1;
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, direction*120);// 30.0); // 120.0);
        }
    }
}


/*
 * Throws string
 */
void NaoBehavior::readSkillsFromFile( const std::string& filename) {
//  cerr << "Loading skills from file " << filename << endl;


    // Load a skill file to memory. Assuming a file is < 4K

    int buffsize = 65536;
    char buff[buffsize];
    int numRead;

    fstream skillFile( filename.c_str(), ios_base::in );
    skillFile.read( buff, buffsize );
    if( !skillFile.eof() ) {
        throw "failed to read the whole skill file " + filename;
    }
    numRead = skillFile.gcount();

    // padding with \0 at the end
    buff[numRead] = '\0';



    // Preprocessing: replace parameters by values.

    string skillDescription("");
    skillDescription.reserve( buffsize );
    for( int i = 0; i < numRead; ++i ) {
        char c = buff[i];
        if( c == '$' ) {
            // parameter - replace it

            string param("");
            i += 1;
            while( i < numRead && ( isalnum( buff[i] ) || buff[i] == '_' ) ) {
                param += buff[i];
                ++i;
            }

            map<string, string>::const_iterator it = namedParams.find( param );
            if( it == namedParams.end() ) {
                throw "Missing parameter in skill file " + filename + ": " + param;
            }
            skillDescription += it->second;

            if( i < numRead )
                skillDescription += buff[i];

        } else {
            // not a param, just concatenate c
            skillDescription += c;

        }
    }




    // Parse

    SkillParser parser( skills, bodyModel );
    parse_info<iterator_t> info = parse( skillDescription.c_str(),
                                         parser,
                                         ( space_p | comment_p("#") )
                                       );



    // check results
    if (info.hit)
    {
//    cout << "-------------------------\n";
//    cout << "Parsing succeeded\n";
//    cout << "-------------------------\n";
//    cout << "stop "  << info.stop << endl;
//    cout << "full " << info.full << endl;
//    cout << "length " << info.length << endl;
    }
    else
    {
        cout << "-------------------------\n";
        cout << "Parsing failed\n";
        //            cout << "stopped at: \": " << info.stop << "\"\n";
        cout << "-------------------------\n";
//    throw "Parsing failed";
    }

}


bool NaoBehavior::isRightSkill( SkillType skill ) {
    string skillStr = EnumParser<SkillType>::getStringFromEnum( skill );
    return skillStr.find("RIGHT") != string::npos;
}

bool NaoBehavior::isLeftSkill( SkillType skill ) {
    string skillStr = EnumParser<SkillType>::getStringFromEnum( skill );
    return skillStr.find("LEFT") != string::npos;
}


double NaoBehavior::
trim(const double& value, const double& min, const double&max)
{
    double ret;
    if (value > max)
        ret = max;
    else if (value < min)
        ret = min;
    else
        ret = value;

    return ret;
}

void NaoBehavior::calculateAngles() {

    float  accX = raw_sensors_->values_[accelX];
    float  accY = raw_sensors_->values_[accelY];
    float  accZ = raw_sensors_->values_[accelZ];

    raw_sensors_->values_[angleX] = atan2(accY,accZ);
    raw_sensors_->values_[angleY] = -atan2(accX,accZ);

    //raw_sensors_->values_[gyroX] = 0; // = 1000000.0;
    //raw_sensors_->values_[gyroY] = 0; //= 1000000.0;
}

void NaoBehavior::preProcessJoints() {
    for (int i=0; i<NUM_JOINTS; i++) {
        processed_joint_angles_->values_[i] = spark_joint_signs[i] * raw_joint_angles_->values_[i];
    }
}

void NaoBehavior::postProcessJoints() {
    raw_joint_commands_->angle_time_ = processed_joint_commands_->angle_time_;
    raw_joint_commands_->stiffness_time_ = processed_joint_commands_->stiffness_time_;
    for (int i=0; i<NUM_JOINTS; i++) {
        raw_joint_commands_->angles_[i] = spark_joint_signs[i] * processed_joint_commands_->angles_[i]; // apply joint signs to convert to the robot's reference frame
        raw_joint_commands_->stiffness_[i] = processed_joint_commands_->stiffness_[i];
    }
    raw_joint_commands_->send_stiffness_ = processed_joint_commands_->send_stiffness_;
    processed_joint_commands_->send_stiffness_ = false;
}

void NaoBehavior::resetSkills() {
    skills[skill]->reset();
    shouldIgnoreVision = false;
    skill = SKILL_STAND;
    isBusyRLKick = false;
    hasStartedRLKick = false;
    skillState = 0;

    kickDirection = VecPosition(1.0, 0, 0);

    resetScales();

    kickType = KICK_FORWARD;

    skills[worldModel->getLastSkill()]->reset();
    worldModel->setLastSkill(skill);

    bodyModel->setUseOmniWalk(true);
}

void NaoBehavior::resetScales() {
    for (int e = int(EFF_H1); e < int(EFF_NUM); e++) {
        bodyModel->setScale(e, 1.0);
    }
}


// Determines whether a collision will occur while moving to a target, adjusting accordingly when necessary
VecPosition NaoBehavior::collisionAvoidance(bool avoidTeammate, bool avoidOpponent, bool avoidBall, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, bool fKeepDistance) {
    // Obstacle avoidance
    VecPosition closestObjPos = VecPosition(100, 100, 0);
    double closestObjDistance = me.getDistanceTo(closestObjPos);

    // Avoid the ball if flag is set
    if(avoidBall) {
        if (abs(me.getAngleBetweenPoints(target, ball)) < 90.0
                && (fKeepDistance || me.getDistanceTo(ball) <= me.getDistanceTo(target))) {
            closestObjPos = ball;
            closestObjDistance = me.getDistanceTo(ball);
        }
    }

    // Avoid all of your teamates if flag is set
    if(avoidTeammate) {
        for(int i = WO_TEAMMATE1; i <= WO_TEAMMATE11; ++i) {
            // Skip ourself
            if (worldModel->getUNum() == i - WO_TEAMMATE1 + 1) {
                continue;
            }
            WorldObject* teammate = worldModel->getWorldObject( i );
            if (teammate->validPosition == true) {
                VecPosition temp = teammate->pos;
                temp.setZ(0);
                if (abs(me.getAngleBetweenPoints(target, temp)) < 90.0) {
                    if (!fKeepDistance && me.getDistanceTo(temp) > me.getDistanceTo(target)) {
                        continue;
                    }
                    double distance = me.getDistanceTo(temp);
                    if (distance < closestObjDistance) {
                        closestObjDistance = distance;
                        closestObjPos = temp;
                    }
                }
            }
        }
    }

    // Avoid opponents if flag is set
    if(avoidOpponent) {
        if (closestObjDistance > PROXIMITY_THRESH) {
            for(int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
                WorldObject* opponent = worldModel->getWorldObject( i );
                if (opponent->validPosition == true) {
                    VecPosition temp = opponent->pos;
                    temp.setZ(0);
                    if (abs(me.getAngleBetweenPoints(target, temp)) < 90.0 &&
                            me.getDistanceTo(temp) < me.getDistanceTo(target)) {
                        double distance = me.getDistanceTo(temp);
                        if (distance < closestObjDistance) {
                            closestObjDistance = distance;
                            closestObjPos = temp;
                        }
                    }
                }
            }
        }
    }

    // Determine where you need to move to avoid the closest object you want to avoid
    if (closestObjDistance <= PROXIMITY_THRESH) {
        VecPosition originalTarget = target;
        target = collisionAvoidanceCorrection(me, PROXIMITY_THRESH, COLLISION_THRESH, target, closestObjPos);
    }

    return target;
}

VecPosition NaoBehavior::collisionAvoidanceCorrection(VecPosition start, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle) {
    double obstacleDist = start.getDistanceTo(obstacle);

    if (abs(start.getAngleBetweenPoints(target, obstacle)) >= 90.0 ||
            obstacleDist > PROXIMITY_THRESH) {
        return target;
    }


    VecPosition obstacleDir = (obstacle-start).normalize();

    VecPosition left90 = start + VecPosition(0, 0, 1).crossProduct(obstacleDir)*1.0;
    VecPosition right90 = start - VecPosition(0, 0, 1).crossProduct(obstacleDir)*1.0;
    if (target.getDistanceTo(left90) > target.getDistanceTo(right90)) {
        target = right90;
    } else {
        target = left90;
    }

    if (obstacleDist <= COLLISION_THRESH) {
        // We're way too close so also back away
        target += (start-obstacle).normalize()*1.0;
    }
    return target;
}

VecPosition NaoBehavior::collisionAvoidanceApproach(double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle) {
    return collisionAvoidanceApproach(me, PROXIMITY_THRESH, COLLISION_THRESH, target, obstacle);
}

VecPosition NaoBehavior::collisionAvoidanceApproach(VecPosition start, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle) {
    double distanceToObstacle = start.getDistanceTo(obstacle);
    if (fabs(start.getAngleBetweenPoints(target, obstacle)) >= 90.0 ||
            distanceToObstacle > start.getDistanceTo(target)) {
        return target;
    }

    if (distanceToObstacle <= PROXIMITY_THRESH) {
        return collisionAvoidanceCorrection(start, PROXIMITY_THRESH, COLLISION_THRESH, target, obstacle);
    }

    VecPosition start2Target = target-start;
    VecPosition start2TargetDir = VecPosition(start2Target).normalize();
    VecPosition start2Obstacle = obstacle-start;
    VecPosition start2ObstacleDir = VecPosition(start2Obstacle).normalize();


    VecPosition closestPathPoint = start+
                                   (start2TargetDir*(start2Obstacle.dotProduct(start2TargetDir)));

    double pathDistanceFromObstacle = (obstacle-closestPathPoint).getMagnitude();
    VecPosition originalTarget = target;
    if (pathDistanceFromObstacle < PROXIMITY_THRESH) {
        target = obstacle + (closestPathPoint-obstacle).normalize()*PROXIMITY_THRESH;
    }
    return target;

}




SkillType NaoBehavior::getWalk(const double& direction, const double& rotation, double speed, bool fAllowOver180Turn)
{
    return getWalk(WalkRequestBlock::PARAMS_DEFAULT, direction, rotation, speed, fAllowOver180Turn);
}

SkillType NaoBehavior::getWalk(WalkRequestBlock::ParamSet paramSet, const double& direction, double rotation, double speed, bool fAllowOver180Turn)
{
    double reqDirection, relSpeed;

    if (worldModel->getTime()-lastGetupRecoveryTime < 1.0 && abs(direction) > 90) {
        // Don't try and walk backwards if we just got up as we are probably unstable
        speed = 0;
    }

    // Convert direction angle to the range [0, 360)
    reqDirection = fmod(direction, 360.0);
    reqDirection += (reqDirection < 0) ? 360 : 0;
    assert((reqDirection >= 0) && (reqDirection <= 360));

    // Trim the relative speed
    relSpeed = trim(speed, 0, 1);

    double tanReqDirection, tanMaxSpeed;
    double maxSpeedX, maxSpeedY, maxRot;

    // Desired velocity and rotation as a percentage of the maximum speed.
    double relSpeedX, relSpeedY, relRot;

    // Get the maximum speed.
    maxSpeedX = core->motion_->getMaxXSpeed(); //core->walkEngine.p.speedMax.translation.x;
    maxSpeedY = core->motion_->getMaxYSpeed(); // core->walkEngine.p.speedMax.translation.y;

    relRot = rotation;
    // There is no reason to request a turn > 180 or < -180 as in that case
    // we should just turn the other way instead
    if (!fAllowOver180Turn) {
        if (relRot > 180) {
            relRot -= 360.0;
        } else if (relRot < -180) {
            relRot += 360.0;
        }
    }

    relRot = rotation / 180;

    // Truncate to (+/-)1.0
    relRot = trim(relRot, -1, 1);

    // Calculate tangent. Due to floating point error and the special way the walk
    // engine treats walk requests with only one non-zero component, it is necessary
    // to explicitly set values for direction requests that are multiples of 90.
    if ((reqDirection == 0) || (reqDirection == 180))
        tanReqDirection = 0;
    else if ((reqDirection == 90) || (reqDirection == 270))
        tanReqDirection = INFINITY;
    else
        tanReqDirection = abs(tanDeg(reqDirection));

    tanMaxSpeed = maxSpeedY / maxSpeedX;

    // Determine the maximum relative speeds that will result in
    // a walk in the appropriate direction.
    if (tanReqDirection < tanMaxSpeed)
    {
        relSpeedX = 1;
        relSpeedY = tanReqDirection / tanMaxSpeed;
    }
    else
    {
        relSpeedX = tanMaxSpeed / tanReqDirection;
        relSpeedY = 1;
    }

    // Get signs correct. Forward is positive X. Left is positive Y.
    if (reqDirection > 180)
        relSpeedY *= -1;

    if ((reqDirection > 90) && (reqDirection < 270))
        relSpeedX *= -1;

    // Abrubt stops or changes in direction can be unstable with the approach
    // ball walk parameter set so check for this and stabilize if need be
    static WalkRequestBlock::ParamSet lastWalkParamSet = WalkRequestBlock::PARAMS_DEFAULT;
    static bool fLastWalkParamRequestWasApproach = false;
    static double lastWalkParamRequestApproachTime = 999999999;
    bool fStabilize = false;
    if (paramSet == WalkRequestBlock::PARAMS_APPROACH_BALL) {
        if (!fLastWalkParamRequestWasApproach) {
            lastWalkParamRequestApproachTime = worldModel->getTime();
        }
        fLastWalkParamRequestWasApproach = true;

        if (lastWalkParamSet != WalkRequestBlock::PARAMS_APPROACH_BALL && (speed < .5 || abs(direction) > 45)) {
            if (worldModel->getTime()-lastWalkParamRequestApproachTime < .5) {
                paramSet = WalkRequestBlock::PARAMS_DEFAULT;
                fStabilize = true;
                relSpeed, relRot = 0;
            }
        }
    } else {
        fLastWalkParamRequestWasApproach = false;
    }

    if (lastWalkParamSet != paramSet) {
        lastWalkParamSet = paramSet;
    }


    // Sanity checks. The absolute value of these variables must be <= 1.
    // However, because of floating point error, it's possible that they are
    // slightly greater than one.
    assert(abs(relSpeedX) < 1.001);
    assert(abs(relSpeedY) < 1.001);
    assert(abs(relRot) < 1.001);


    // Record the desired velocity and return the SKILL_WALK_OMNI.
    // NaoBehavior::act() will use the speed components in velocity
    // generate a request to the omnidirectional walk engine whenever the
    // SKILL_WALK_OMNI is invoked.
    velocity = WalkVelocity(paramSet, relSpeed * relSpeedX, relSpeed * relSpeedY, relRot);

    if (fStabilize) {
        // Stabilize
        return SKILL_STAND;
    }

    return SKILL_WALK_OMNI;
}

// Currently untuned. For example, there's no slow down...
SkillType NaoBehavior::goToTargetRelative(const VecPosition& targetLoc, const double& targetRot, const double speed, bool fAllowOver180Turn, WalkRequestBlock::ParamSet paramSet)
{
    double walkDirection, walkRotation, walkSpeed;

    walkDirection = targetLoc.getTheta();
    walkRotation = targetRot;
    walkSpeed = speed;

    walkSpeed = trim(walkSpeed, 0.1, 1);

    if (targetLoc.getMagnitude() == 0)
        walkSpeed = 0;

    return getWalk(paramSet, walkDirection, walkRotation, walkSpeed, fAllowOver180Turn);
}


//Assumes target = z-0. Maybe needs further tuning
SkillType NaoBehavior::goToTarget(const VecPosition &target) {
    double distance, angle;
    getTargetDistanceAndAngle(target, distance, angle);

    const double distanceThreshold = 1;
    const double angleThreshold = getLimitingAngleForward() * .9;
    VecPosition relativeTarget = VecPosition(distance, angle, 0, POLAR);

    // Turn to the angle we want to walk in first, since we want to walk with
    // maximum forwards speeds if possible.
    /*if (abs(angle) > angleThreshold)
    {
    return goToTargetRelative(VecPosition(), angle);
    }*/

    // [patmac] Speed/quickness adjustment
    // For now just go full speed in the direction of the target and also turn
    // toward our heading.
    SIM::AngDeg turnAngle = angle;

    // If we are within distanceThreshold of the target, we walk directly to the target
    if (me.getDistanceTo(target) < distanceThreshold) {
        turnAngle = 0;
    }

    // Walk in the direction that we want.
    return goToTargetRelative(relativeTarget, turnAngle);
}

double NaoBehavior::getLimitingAngleForward() {
    double maxSpeedX = core->motion_->getMaxXSpeed(); //core->walkEngine.p.speedMax.translation.x;
    double maxSpeedY = core->motion_->getMaxYSpeed(); // core->walkEngine.p.speedMax.translation.y;
    return abs(atan2Deg(maxSpeedY, maxSpeedX));
}


void NaoBehavior::refresh() {
    myXDirection = worldModel->l2g(VecPosition(1.0, 0, 0)) - worldModel->l2g(VecPosition(0, 0, 0));
    myXDirection.setZ(0);
    myXDirection.normalize();

    myYDirection = worldModel->l2g(VecPosition(0, 1.0, 0)) - worldModel->l2g(VecPosition(0, 0, 0));
    myYDirection.setZ(0);
    myYDirection.normalize();

    //Anomalous
    myZDirection = worldModel->l2g(VecPosition(0, 0, 1.0)) - worldModel->l2g(VecPosition(0, 0, 0));
    myZDirection.normalize();

    me = worldModel->getMyPosition(); // ->l2g(VecPosition(0, 0, 0)); // <- had consistency problems
    me.setZ(0);

    ball = worldModel->getBall();
    ball.setZ(0);
}



//Assumes target it z-0.
void NaoBehavior::getTargetDistanceAndAngle(const VecPosition &target, double &distance, double &angle) {
    VecPosition targetDirection = VecPosition(target) - me;
    targetDirection.setZ(0);

    // distance
    distance = targetDirection.getMagnitude();

    // angle
    targetDirection.normalize();

    angle = VecPosition(0, 0, 0).getAngleBetweenPoints(myXDirection, targetDirection);
    if (isnan(angle)) {
        //cout << "BAD angle!\n";
        angle = 0;
    }
    if(myYDirection.dotProduct(targetDirection) < 0) {
        angle = -angle;
    }
}


bool NaoBehavior::beamablePlayMode() {
    int pm = worldModel->getPlayMode();
    return pm == PM_BEFORE_KICK_OFF || pm == PM_GOAL_LEFT || pm == PM_GOAL_RIGHT;
}

bool NaoBehavior::improperPlayMode() {
    return improperPlayMode(worldModel->getPlayMode());
}

/* Playmodes when we can't touch the ball or game is over (it's not proper to do so) */
bool NaoBehavior::improperPlayMode(int pm) {

    if(pm == PM_BEFORE_KICK_OFF) {
        return true;
    }
    else if(pm == PM_GAME_OVER) {
        return true;
    }
    else if(pm == PM_GOAL_LEFT) {
        return true;
    }
    else if(pm == PM_GOAL_RIGHT) {
        return true;
    }

    if(worldModel->getSide() == SIDE_LEFT) {

        if(pm == PM_KICK_OFF_RIGHT) {
            return true;
        }
        else if(pm == PM_KICK_IN_RIGHT) {
            return true;
        }
        else if(pm == PM_CORNER_KICK_RIGHT) {
            return true;
        }
        else if(pm == PM_GOAL_KICK_RIGHT) {
            return true;
        }
        else if(pm == PM_OFFSIDE_LEFT) {
            return true;
        }
        else if(pm == PM_FREE_KICK_RIGHT) {
            return true;
        }
        else if(pm == PM_DIRECT_FREE_KICK_RIGHT) {
            return true;
        }
        else if(pm == PM_PASS_RIGHT) {
            return true;
        }
    }
    else if(worldModel->getSide() == SIDE_RIGHT) {

        if(pm == PM_KICK_OFF_LEFT) {
            return true;
        }
        else if(pm == PM_KICK_IN_LEFT) {
            return true;
        }
        else if(pm == PM_CORNER_KICK_LEFT) {
            return true;
        }
        else if(pm == PM_GOAL_KICK_LEFT) {
            return true;
        }
        else if(pm == PM_OFFSIDE_RIGHT) {
            return true;
        }
        else if(pm == PM_FREE_KICK_LEFT) {
            return true;
        }
        else if(pm == PM_DIRECT_FREE_KICK_LEFT) {
            return true;
        }
        else if(pm == PM_PASS_LEFT) {
            return true;
        }
    }

    return false;
}

bool NaoBehavior::kickPlayMode() {
    return kickPlayMode(worldModel->getPlayMode());
}

bool NaoBehavior::kickPlayMode(int pm, bool eitherTeam) {
    if(!eitherTeam && improperPlayMode(pm)) {
        return false;
    }

    return pm == PM_CORNER_KICK_LEFT || pm == PM_CORNER_KICK_RIGHT || pm == PM_KICK_IN_LEFT || pm == PM_KICK_IN_RIGHT || pm == PM_FREE_KICK_LEFT || pm == PM_FREE_KICK_RIGHT || pm == PM_DIRECT_FREE_KICK_LEFT || pm == PM_DIRECT_FREE_KICK_RIGHT || pm == PM_GOAL_KICK_LEFT || pm == PM_GOAL_KICK_RIGHT || pm == PM_KICK_OFF_LEFT || pm == PM_KICK_OFF_RIGHT;
}

bool NaoBehavior::isIndirectKick() {
    return isIndirectKick(worldModel->getPlayMode());
}

bool NaoBehavior::isIndirectKick(int pm) {
    if(!kickPlayMode(pm, true)) {
        return false;
    }

    return !(pm == PM_DIRECT_FREE_KICK_LEFT || pm == PM_DIRECT_FREE_KICK_RIGHT || pm == PM_CORNER_KICK_LEFT || pm == PM_CORNER_KICK_RIGHT || pm == PM_GOAL_KICK_LEFT || pm == PM_GOAL_KICK_RIGHT);
}


/* Set message to be send to the monitor port */
void NaoBehavior::setMonMessage(const std::string& msg) {
    monMsg = msg;
}

/* Get message to be sent to the monitor port.  Also flushes message */
string NaoBehavior::getMonMessage() {
    string ret = monMsg;
    monMsg = "";
    return ret;
}

bool NaoBehavior::HasMadeAPass(VecPosition initial_ball_pos)
{

    //if (m_is_receiver)
      // return false;

    if (initial_ball_pos.getX() == -1 || initial_ball_pos.getY() == -1
    || initial_ball_pos.getZ() == -1 )
      return false;
     //  It probably means the player made a pass
    if (ball.getDistanceTo(initial_ball_pos) >0.5  )
    {
        // 
        return true;
    }
     // re
    return false;
}
