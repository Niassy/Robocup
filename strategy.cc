
#include "naobehavior.h"
#include "../rvdraw/rvdraw.h"
#include "scram.h"
#include "rl/utils.h"
#include <string>
#include <vector>
#include <chrono>
//#include "scram.h"
extern int agentBodyType;

#ifdef IS_RL_TRAIN
extern int RLPort;
#endif


using namespace std::chrono;
extern int agentBodyType;
extern bool fFatProxy;


#ifdef IS_MEASURING_KICKS
    int mike_measure_kick_counter = 0;
    int mike_measure_current_kick_idx = 0;
    int mike_measure_max_kicks = 3;
    std::vector<std::string> mike_measure_kick_types_to_run    = {"long", "normal"};
    std::vector<std::vector<double>> mike_measure_kick_distance_scores(mike_measure_kick_types_to_run.size(), std::vector<double>(mike_measure_max_kicks, 0.0));
    VecPosition kickStartLoc(0,0,0);
#endif

/*
 * Real game beaming.
 * Filling params x y angle
 */
void NaoBehavior::beam( double& beamX, double& beamY, double& beamAngle ) {
    //cout << "PNUM: "<< worldModel->getUNum() << " GMODE: " << worldModel->getPlayMode() << " X:" << beamX << " Y:" << beamY << endl; 
    //if(worldModel->getPlayMode() == 0 || (worldModel->getPlayMode() == 6) || (worldModel->getPlayMode() == 7)){

        if(worldModel->getUNum() == 1){

		    beamX = -HALF_FIELD_X;
            beamY = 0;
            beamAngle = 0;
        }
        else if(worldModel->getUNum() == 2){
            // VecPosition Pos2 = VecPosition(0,0,0).Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
		    beamX = -3; //Pos2.getX() + 1.5;
            beamY = -1; //Pos2.getY();
            beamAngle = 0;
        }
        else if(worldModel->getUNum() == 3){
            VecPosition Pos2 = VecPosition(0,0,0).Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            VecPosition Pos3 = VecPosition(Pos2.getX()-1, Pos2.getY()+1, 0);
		    beamX = Pos3.getX();
            beamY = Pos3.getY();
            beamAngle = 0;
        }
        else if(worldModel->getUNum() == 4){
            VecPosition Pos2 = VecPosition(0,0,0).Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            VecPosition Pos4 = VecPosition(Pos2.getX()-1, Pos2.getY()-1, 0);
		    beamX = Pos4.getX();
            beamY = Pos4.getY();
            beamAngle = 0;
        }
        else if(worldModel->getUNum() == 5){
            VecPosition Pos5 = VecPosition(VecPosition(0,0,0).getX()-2, VecPosition(0,0,0).getY()+3 , 0);
		    beamX = Pos5.getX();
            beamY = Pos5.getY();
            beamAngle = 0;
        }
        else if(worldModel->getUNum() == 6){
            VecPosition Pos6 =  VecPosition(VecPosition(0,0,0).getX() - 0.8, VecPosition(0,0,0).getY()-2.1 , 0);
		    beamX = Pos6.getX();
            beamY = Pos6.getY();
            beamAngle = 0;
        }
        else if(worldModel->getUNum() == 7){
            VecPosition Pos7 = VecPosition(VecPosition(0,0,0).getX()-7, VecPosition(0,0,0).getY(),0);
		    beamX = Pos7.getX();
            beamY = Pos7.getY();
            beamAngle = 0;
        }
        else if(worldModel->getUNum() == 8){
            VecPosition Pos2 = VecPosition(0,0,0).Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            VecPosition Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
		    beamX = Pos8.getX();
            beamY = Pos8.getY();
            beamAngle = 0;
        }
        else if(worldModel->getUNum() == 9){
            VecPosition Pos2 = VecPosition(0,0,0).Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            VecPosition Pos9 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9.setX(Pos9.getX()-1);
		    beamX = Pos9.getX();
            beamY = Pos9.getY();
            beamAngle = 0;
        }
        else if(worldModel->getUNum() == 10){
            VecPosition Pos10 =  VecPosition(-5.5, 5 , 0);
		    beamX = Pos10.getX();
            beamY = Pos10.getY();
            beamAngle = 0;
        }
        else if(worldModel->getUNum() == 11){
            VecPosition Pos11 = VecPosition(-5, -5,0);
		    beamX = Pos11.getX();
            beamY = Pos11.getY();
            beamAngle = 0;
        }
    //beamX = ball.getX()-0.2;
    //beamY = ball.getY();
    //beamAngle = 0;

}

SkillType NaoBehavior::goalKeeperBehaviour(){

    SIM::Point2D ball2Point2D = SIM::Point2D(ball.getX(), ball.getY());
    SIM::Point2D goalLine = SIM::Point2D(-HALF_FIELD_X, 0);
    SIM::Point2D position = ball2Point2D.getPointOnLineFraction(goalLine, 0.9);
    double velX = worldModel->getWorldObject(WO_BALL)->absVel.getX();
    double velY = worldModel->getWorldObject(WO_BALL)->absVel.getY();
    pair<VecPosition, double> t0, tn;
    SIM::Point2D velocity0, velocityN;

    static queue<pair<VecPosition,double>> history;
    static queue<SIM::Point2D> velHistory;

    VecPosition target = VecPosition(position.getX(), position.getY(), 0);

    if(history.size() < 120) {
        history.push(make_pair(ball, worldModel->getGameTime()));
        velHistory.push(worldModel->getWorldObject(WO_BALL)->absVel);
    } else  {
       t0 = history.front();
       tn = history.back();
       velocity0 = velHistory.front();
       velocityN = velHistory.back();
       history.pop();
       history.push(make_pair(ball, worldModel->getGameTime()));
       velHistory.pop();
       velHistory.push(worldModel->getWorldObject(WO_BALL)->absVel);
    //    cout << t0.second << " " << tn.second << endl;
    }
    
    if(worldModel->canTrustVision()) {
        if( abs(velX) > 0.05 || abs(velY) > 0.05){
            // cout << worldModel->getWorldObject(WO_BALL)->absVel << endl;
            double x, y, a, t;
            t = tn.second - t0.second;
            a = (velocityN.getX() + velocity0.getX())/t;
            x = tn.first.getX() + velocityN.getX()*t + (1/2)*a*pow(t,2);
            y = tn.first.getY() + velocityN.getY()*t + (1/2)*a*pow(t,2);


            //worldModel->getRVSender()->drawCircle("Predicted", x, y,0.1,1,0.5,0); //ORANGE
            //worldModel->getRVSender()->drawCircle("Predicted1",ball.getX(), ball.getY(),0.05,1,0,0); //RED
        
            if(ball.getX() < worldModel->getMyPosition().getX()) {
                return kickBall(KICK_IK, VecPosition(-HALF_FIELD_X, 3, 0));
            } 
            if(x < worldModel->getMyPosition().getX()) {
                if(y < worldModel->getMyPosition().getY()) {
                    return SKILL_LATERAL_DIVE_LEFT;
                } else {
                    return SKILL_LATERAL_DIVE_RIGHT;
                }
                
            }
        }
        float distance = worldModel->getMyPosition().getDistanceTo(target);
        if(distance < 0.5){
            //worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
            return FaceBall(worldModel->getMyPosition());
        } else {
            return goToTarget(target);
        }
    } else {
         return SKILL_STAND;
    }

    //cout << worldModel->getWorldObject(WO_BALL)->absVel << endl;
    return SKILL_STAND;
}

vector<VecPosition> NaoBehavior::GenerateImportantPositions(int _playMode, int _side, int _playerNumber, int regionX, int regionY){


    vector<VecPosition> ImportantPositions;
    std::string SpNum = std::to_string(_playerNumber);
    double offset = 0;
    SIM::Point2D ballp = SIM::Point2D(ball.getX(), ball.getY());
    SIM::Point2D orgoal = SIM::Point2D(-HALF_FIELD_X,0);
    SIM::Point2D offp = ballp.getPointOnLineFraction(orgoal,offset);
    VecPosition offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 
    VecPosition MidBoalGoal = ball.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));

    VecPosition Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
    VecPosition Pos2 = ball.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0)); // Middle of X formation of defenders
    VecPosition Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
    VecPosition Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
    VecPosition Pos5 = VecPosition(Pos2.getX()+2, Pos2.getY()+2, 0); // Forward Up from mid X
    VecPosition Pos6 = VecPosition(Pos2.getX()+2, Pos2.getY()-2, 0); // Forward DOwn from mid X
    VecPosition Pos7 = VecPosition(ball.getX()+ 3, ball.getY(),0); // Infront of ball

    VecPosition Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
    VecPosition Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
    VecPosition Pos10 = ball.Midpoint(Pos2);
    Pos10.setY(Pos10.getY()+1);
    VecPosition Pos11 = ball.Midpoint(Pos2);
    Pos11.setY(Pos11.getY()-1);

    
    if(_playMode == PM_BEFORE_KICK_OFF){
        Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
        Pos2 = MidBoalGoal; // Middle of X formation of defenders
        Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
        Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
        Pos5 = VecPosition(ball.getX()-0.3, ball.getY()+3, 0); // Forward Up from ball
        Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-3, 0); // Forward DOwn from ball
        Pos7 = VecPosition(ball.getX()-4, ball.getY(),0); // Behind ball

        Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
        Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
        Pos10 = ball.Midpoint(Pos2);
        Pos10.setY(Pos10.getY()+1);
        Pos11 = ball.Midpoint(Pos2);
        Pos11.setY(Pos11.getY()-1);


    }
    else if(_playMode == PM_KICK_OFF_LEFT){
        if(_side == SIDE_LEFT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = VecPosition(-3, -1, 0); //MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(MidBoalGoal.getX()-2, MidBoalGoal.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(MidBoalGoal.getX()-2, MidBoalGoal.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(1, 3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-2, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-7, ball.getY(),0); // On the ball

            Pos8 = MidBoalGoal.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = VecPosition(ball.getX()+3, ball.getY()+4, 0); // Forward Up from ball
            Pos11 = VecPosition(ball.getX()+5, ball.getY()-2, 0); // Forward Up from ball
        }
        else{
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-0.3, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = ball.Midpoint(Pos2);
            Pos10.setY(Pos10.getY()+5);
            Pos11 = ball.Midpoint(Pos2);
            Pos11.setY(Pos11.getY()-5);
        }
    }
    else if(_playMode == PM_KICK_OFF_RIGHT){
        if(_side== SIDE_RIGHT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = VecPosition(-3, -1, 0); // Middle of X formation of defenders
            Pos3 = VecPosition(MidBoalGoal.getX()-2, MidBoalGoal.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(MidBoalGoal.getX()-2, MidBoalGoal.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(1, 3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-2, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-7, ball.getY(),0); // On the ball

            Pos8 = MidBoalGoal.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = VecPosition(ball.getX()+3, ball.getY()+4, 0); // Forward Up from ball
            Pos11 = VecPosition(ball.getX()+5, ball.getY()-2, 0); // Forward Up from ball


        }
        else{
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-0.3, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = ball.Midpoint(Pos2);
            Pos10.setY(Pos10.getY()+5);
            Pos11 = ball.Midpoint(Pos2);
            Pos11.setY(Pos11.getY()-5);
        }
    }
    
    else if((_playMode == PM_FREE_KICK_LEFT) || (_playMode == PM_KICK_IN_LEFT) || (_playMode == PM_GOAL_KICK_LEFT)){
        if(_side == SIDE_LEFT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.2,0,0); // Our Goal
            Pos2 = VecPosition(-HALF_FIELD_X/2,0,0); // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos6 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos7 = VecPosition(Pos2.getX()+5, ball.getY(),0); // On ball
            Pos8 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos9 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos10 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos11 = VecPosition(-HALF_FIELD_X+3,0,0); // On ball
        }
        else {
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-2, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-2, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball
        }
    }
    else if((_playMode == PM_FREE_KICK_RIGHT) || (_playMode == PM_KICK_IN_RIGHT) || (_playMode == PM_GOAL_KICK_RIGHT)){
        if(worldModel->getSide() == SIDE_RIGHT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.2,0,0); // Our Goal
            Pos2 = VecPosition(-HALF_FIELD_X/2,0,0); // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos6 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos7 = VecPosition(Pos2.getX()+5, ball.getY(),0); // On ball
            Pos8 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos9 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos10 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos11 = VecPosition(-HALF_FIELD_X+3,0,0); // On ball
        }
        else {
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-2, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-2, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball
        }
        
    }
    else{ // PM_PLAY_ON 

        double yValue = MapValueInRangeTo(ball.getY(), -HALF_FIELD_Y, HALF_FIELD_Y, -HALF_GOAL_Y, HALF_GOAL_Y);

        Pos1 = VecPosition(-HALF_FIELD_X+0.3,yValue,0); // Our Goal
        Pos2 = MidBoalGoal; // Middle of X formation of defenders
        Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
        Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
        Pos5 = VecPosition(Pos2.getX()+2, Pos2.getY()+2, 0); // Forward Up from mid X
        Pos6 = VecPosition(Pos2.getX()+2, Pos2.getY()-2, 0); // Forward DOwn from mid X
        Pos7 = VecPosition(ball.getX(), ball.getY(),0); // Infront of ball

        if(regionX ==1){
            if(regionY ==1){

                offset = 0.2;
                offp = ballp.getPointOnLineFraction(orgoal,offset);
                offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                Pos2 = offsetPos; // Middle of X formation of defenders
                Pos4 = VecPosition(Pos2.getX()-1, Pos2.getY()-1, 0); // Back DOwn from mid X
                Pos6 = VecPosition(Pos2.getX()+1, Pos2.getY()-1, 0); // Forward DOwn from mid X

                Pos3 = VecPosition(0, 5, 0);
                Pos5 = VecPosition(5,0,0); 
                Pos7 = VecPosition(7,4,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(ball.getX()+5, ball.getY(), 0);
                Pos11 = VecPosition(-5, 0, 0);
            }
            else if(regionY ==2){

                //this is our middle block region containing our goal

                if(BallinMyGoalMouth()){
                    offset = 0.08;
                    offp = ballp.getPointOnLineFraction(orgoal,offset);
                    offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                    Pos2 = offsetPos; // Middle of X formation of defenders
                    Pos3 = VecPosition(Pos1.getX()+0.2, Pos1.getY()+0.6, 0); // Back Up from mid X
                    Pos4 = VecPosition(Pos1.getX()+0.2, Pos1.getY()-0.6, 0); // Back DOwn from mid X


                    ///play downwards
                    Pos6 =  VecPosition(-5, -4, 0);
                    ///play upwards
                    Pos5 =  VecPosition(-5, 4, 0);
                    Pos7 = VecPosition(-5,0,0);

                    Pos8 = VecPosition(5, 0, 0);
                    Pos9 = VecPosition(-HALF_FIELD_X+5, 0, 0);
                    Pos10 = VecPosition(-HALF_FIELD_X+2, 2.5, 0);
                    Pos11 = VecPosition(-HALF_FIELD_X+2, -2.5, 0);
                }
                else{

                    offset = 0.08;
                    offp = ballp.getPointOnLineFraction(orgoal,offset);
                    offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                    Pos2 = offsetPos; // Middle of X formation of defenders
                    Pos3 = VecPosition(Pos2.getX()-0.5, Pos2.getY()+0.5, 0); // Back Up from mid X
                    Pos4 = VecPosition(Pos2.getX()-0.5, Pos2.getY()-0.5, 0); // Back DOwn from mid X


                    ///play downwards
                    Pos6 =  VecPosition(ball.getX()+4, ball.getY()-4, 0);
                    ///play upwards
                    Pos5 =  VecPosition(ball.getX()+4, ball.getY()+4, 0);
                    Pos7 = VecPosition(5,0,0);

                    Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos10 = VecPosition(ball.getX()+5.5, ball.getY(), 0);
                    Pos11 = VecPosition(Pos8.getX()+0.5, Pos8.getY()-2, 0);
                }


            }
            else{

                offset = 0.2;
                offp = ballp.getPointOnLineFraction(orgoal,offset);
                offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                Pos2 = offsetPos; // Middle of X formation of defenders
                Pos3 = VecPosition(Pos2.getX()-1, Pos2.getY()+1, 0); // Back Up from mid X
                Pos5 = VecPosition(Pos2.getX()+1, Pos2.getY()+1, 0); // Forward Up from mid X

                Pos4 = VecPosition(0, -5, 0);
                Pos6 = VecPosition(5,-8,0);
                Pos7 = VecPosition(7,-4,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(-5, -5, 0);
                Pos11 = VecPosition(-5, 0, 0);
            }
        }
        else if(regionX ==2){

            offset = 0.1;
            offp = ballp.getPointOnLineFraction(orgoal,offset);
            offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

            Pos2 = offsetPos; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X

            float dist = VecPosition(0,0,0).getDistanceTo(ball);
            if(dist < 0.2){
                Pos5 = VecPosition(5, 4, 0);   
                Pos6 = VecPosition(0, 5, 0); 
                Pos7 = VecPosition(8, 3,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(Pos2.getX()-3, Pos2.getY()-7, 0);
                Pos11 = VecPosition(Pos2.getX()+3, Pos2.getY()-3, 0);
            }
            else{
                if(ball.getY() > 0){
                Pos5 = VecPosition(5, 4, 0);   
                Pos6 = VecPosition(0, 5, 0); 
                Pos7 = VecPosition(8, 3,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(Pos2.getX()-3, Pos2.getY()-7, 0);
                Pos11 = VecPosition(Pos2.getX()+3, Pos2.getY()-3, 0);

                }
                else{
                    Pos5 = VecPosition(5, -4, 0);
                    Pos6 = VecPosition(0, -5, 0); 
                    Pos7 = VecPosition(8, -3,0);

                    Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos10 = VecPosition(Pos2.getX()-3, Pos2.getY()+7, 0);
                    Pos11 = VecPosition(Pos2.getX()+3, Pos2.getY()+3, 0);

                }
            }

            
        }
        else if(regionX ==3){

            offset = 0.2;
            offp = ballp.getPointOnLineFraction(orgoal,offset);
            offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

            Pos2 = offsetPos; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X


            if(ball.getY() > 0){
                Pos5 = VecPosition(5, 5, 0);
                Pos6 = VecPosition(-2, -7, 0);
                Pos7 = VecPosition(8,3,0);

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(ball.getX()+1, ball.getY()-3, 0);
                Pos11 = VecPosition(7, -1, 0);
            }
            else{
                Pos5 = VecPosition(5, -5, 0);
                Pos6 = VecPosition(-2, 7, 0);
                Pos7 = VecPosition(8,-3,0);

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(ball.getX()+1, ball.getY()+3, 0);
                Pos11 = VecPosition(7, 1, 0);
            }
        }
        else{

            offset = 0.1;
            offp = ballp.getPointOnLineFraction(orgoal,offset);
            offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

            Pos2 = offsetPos; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X

            // Pos5 = VecPosition(9, 0, 0);
            // Pos6 = VecPosition(12, 2, 0);
            // Pos7 = VecPosition(12,-2,0);
            // mike/branden 23/07/03
            Pos5 = VecPosition(8, 0, 0);
            Pos6 = VecPosition(11, 2, 0);
            Pos7 = VecPosition(11,-2,0);

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = VecPosition(0, 0, 0);
            if(regionY ==1){
                Pos10 = VecPosition(5, 7, 0);
                Pos11 = VecPosition(5, -5, 0);
            }
            else if(regionY ==2){
                Pos10 = VecPosition(ball.getX()-5, 5, 0);
                Pos11 = VecPosition(ball.getX()-5, -5, 0);
            }
            else{
                Pos10 = VecPosition(5, -7, 0);
                Pos11 = VecPosition(5, 5, 0);
            }
            

        }




    }

    

    

    //Pos1 = ValidateFieldPos(Pos1);
    Pos2 = ValidateFieldPos(Pos2);
    Pos3 = ValidateFieldPos(Pos3);
    Pos4 = ValidateFieldPos(Pos4);
    Pos5 = ValidateFieldPos(Pos5);
    Pos6 = ValidateFieldPos(Pos6);
    Pos7 = ValidateFieldPos(Pos7);
    Pos8 = ValidateFieldPos(Pos8);
    Pos9 = ValidateFieldPos(Pos9);
    Pos10 = ValidateFieldPos(Pos10);
    Pos11 = ValidateFieldPos(Pos11);

    if(visualise){
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos1.getX(), Pos1.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos2.getX(), Pos2.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos3.getX(), Pos3.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos4.getX(), Pos4.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos5.getX(), Pos5.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos6.getX(), Pos6.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos7.getX(), Pos7.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos8.getX(), Pos8.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos9.getX(), Pos9.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos10.getX(), Pos10.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos11.getX(), Pos11.getY(),0.2,1,1,1); //WHITE 
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,offsetPos.getX(),offsetPos.getY(),0.2,1,0,1); //PINK
    }

    ImportantPositions.push_back(Pos1);
    ImportantPositions.push_back(Pos2);
    ImportantPositions.push_back(Pos3);
    ImportantPositions.push_back(Pos4);
    ImportantPositions.push_back(Pos5);
    ImportantPositions.push_back(Pos6);
    ImportantPositions.push_back(Pos7);
    ImportantPositions.push_back(Pos8);
    ImportantPositions.push_back(Pos9);
    ImportantPositions.push_back(Pos10);
    ImportantPositions.push_back(Pos11);

    

    return ImportantPositions;
}

vector<VecPosition> NaoBehavior::GenerateImportantPositionsFAT(int _playMode, int _side, int _playerNumber, int regionX, int regionY){


    vector<VecPosition> ImportantPositions;
    std::string SpNum = std::to_string(_playerNumber);
    double offset = 0;
    SIM::Point2D ballp = SIM::Point2D(ball.getX(), ball.getY());
    SIM::Point2D orgoal = SIM::Point2D(-HALF_FIELD_X,0);
    SIM::Point2D offp = ballp.getPointOnLineFraction(orgoal,offset);
    VecPosition offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 
    VecPosition MidBoalGoal = ball.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));

    VecPosition Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
    VecPosition Pos2 = ball.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0)); // Middle of X formation of defenders
    VecPosition Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
    VecPosition Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
    VecPosition Pos5 = VecPosition(Pos2.getX()+2, Pos2.getY()+2, 0); // Forward Up from mid X
    VecPosition Pos6 = VecPosition(Pos2.getX()+2, Pos2.getY()-2, 0); // Forward DOwn from mid X
    VecPosition Pos7 = VecPosition(ball.getX()+ 3, ball.getY(),0); // Infront of ball

    VecPosition Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
    VecPosition Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
    VecPosition Pos10 = ball.Midpoint(Pos2);
    Pos10.setY(Pos10.getY()+1);
    VecPosition Pos11 = ball.Midpoint(Pos2);
    Pos11.setY(Pos11.getY()-1);

    
    if(_playMode == PM_BEFORE_KICK_OFF){
        Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
        Pos2 = MidBoalGoal; // Middle of X formation of defenders
        Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
        Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
        Pos5 = VecPosition(ball.getX()-0.3, ball.getY()+3, 0); // Forward Up from ball
        Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-3, 0); // Forward DOwn from ball
        Pos7 = VecPosition(ball.getX()-4, ball.getY(),0); // Behind ball

        Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
        Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
        Pos10 = ball.Midpoint(Pos2);
        Pos10.setY(Pos10.getY()+1);
        Pos11 = ball.Midpoint(Pos2);
        Pos11.setY(Pos11.getY()-1);


    }
    else if(_playMode == PM_KICK_OFF_LEFT){
        if(_side == SIDE_LEFT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-2, ball.getY()+2, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-2, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-4, ball.getY(),0); // On the ball

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = VecPosition(ball.getX()-3, ball.getY()+4, 0); // Forward Up from ball
            Pos11 = VecPosition(ball.getX()-5, ball.getY()+8, 0); // Forward Up from ball
        }
        else{
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-0.3, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = ball.Midpoint(Pos2);
            Pos10.setY(Pos10.getY()+5);
            Pos11 = ball.Midpoint(Pos2);
            Pos11.setY(Pos11.getY()-5);
        }
    }
    else if(_playMode == PM_KICK_OFF_RIGHT){
        if(_side== SIDE_RIGHT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-2, ball.getY()+2, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-2, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-4.5, ball.getY(),0); // On the ball

            Pos8 = VecPosition(5, 0, 0);
            Pos9 = VecPosition(-HALF_FIELD_X+5, 0, 0);
            Pos10 = VecPosition(-HALF_FIELD_X+2, 2.5, 0);
            Pos11 = VecPosition(-HALF_FIELD_X+2, -2.5, 0);
            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = VecPosition(ball.getX()-3, ball.getY()+4, 0); // Forward Up from ball
            Pos11 = VecPosition(ball.getX()-5, ball.getY()+8, 0); // Forward Up from ball


        }
        else{
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-0.3, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = ball.Midpoint(Pos2);
            Pos10.setY(Pos10.getY()+5);
            Pos11 = ball.Midpoint(Pos2);
            Pos11.setY(Pos11.getY()-5);
        }
    }
    
    else if((_playMode == PM_FREE_KICK_LEFT) || (_playMode == PM_KICK_IN_LEFT) || (_playMode == PM_GOAL_KICK_LEFT)){
        if(_side == SIDE_LEFT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.2,0,0); // Our Goal
            Pos2 = VecPosition(-HALF_FIELD_X/2,0,0); // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos6 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos7 = VecPosition(Pos2.getX()+5, ball.getY(),0); // On ball
            Pos8 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos9 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos10 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos11 = VecPosition(-HALF_FIELD_X+3,0,0); // On ball
        }
        else {
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-2, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-2, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball
        }
    }
    else if((_playMode == PM_FREE_KICK_RIGHT) || (_playMode == PM_KICK_IN_RIGHT) || (_playMode == PM_GOAL_KICK_RIGHT)){
        if(worldModel->getSide() == SIDE_RIGHT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.2,0,0); // Our Goal
            Pos2 = VecPosition(-HALF_FIELD_X/2,0,0); // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos6 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos7 = VecPosition(Pos2.getX()+5, ball.getY(),0); // On ball
            Pos8 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos9 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos10 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos11 = VecPosition(-HALF_FIELD_X+3,0,0); // On ball
        }
        else {
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-2, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-2, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball
        }
        
    }
    else{ // PM_PLAY_ON 

        double yValue = MapValueInRangeTo(ball.getY(), -HALF_FIELD_Y, HALF_FIELD_Y, -HALF_GOAL_Y, HALF_GOAL_Y);

        Pos1 = VecPosition(-HALF_FIELD_X+0.3,yValue,0); // Our Goal
        Pos2 = MidBoalGoal; // Middle of X formation of defenders
        Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
        Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
        Pos5 = VecPosition(Pos2.getX()+2, Pos2.getY()+2, 0); // Forward Up from mid X
        Pos6 = VecPosition(Pos2.getX()+2, Pos2.getY()-2, 0); // Forward DOwn from mid X
        Pos7 = VecPosition(ball.getX(), ball.getY(),0); // Infront of ball

        if(regionX ==1){
            if(regionY ==1){

                offset = 0.2;
                offp = ballp.getPointOnLineFraction(orgoal,offset);
                offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                Pos2 = offsetPos; // Middle of X formation of defenders
                Pos4 = VecPosition(Pos2.getX()-1, Pos2.getY()-1, 0); // Back DOwn from mid X
                Pos6 = VecPosition(Pos2.getX()+1, Pos2.getY()-1, 0); // Forward DOwn from mid X

                Pos3 = VecPosition(0, 5, 0);
                Pos5 = VecPosition(5,8,0); 
                Pos7 = VecPosition(7,4,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(-5, 5, 0);
                Pos11 = VecPosition(-5, 0, 0);
            }
            else if(regionY ==2){

                //this is our middle block region containing our goal

                if(BallinMyGoalMouth()){
                    offset = 0.08;
                    offp = ballp.getPointOnLineFraction(orgoal,offset);
                    offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                    Pos2 = offsetPos; // Middle of X formation of defenders
                    Pos3 = VecPosition(Pos1.getX()+0.2, Pos1.getY()+0.6, 0); // Back Up from mid X
                    Pos4 = VecPosition(Pos1.getX()+0.2, Pos1.getY()-0.6, 0); // Back DOwn from mid X


                    ///play downwards
                    Pos6 =  VecPosition(-3, -7, 0);
                    ///play upwards
                    Pos5 =  VecPosition(-3, 6, 0);
                    Pos7 = VecPosition(-5,0,0);

                    Pos8 = VecPosition(5, 0, 0);
                    Pos9 = VecPosition(-HALF_FIELD_X+5, 0, 0);
                    Pos10 = VecPosition(-HALF_FIELD_X+2, 2.5, 0);
                    Pos11 = VecPosition(-HALF_FIELD_X+2, -2.5, 0);
                }
                else{

                    offset = 0.08;
                    offp = ballp.getPointOnLineFraction(orgoal,offset);
                    offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                    Pos2 = offsetPos; // Middle of X formation of defenders
                    Pos3 = VecPosition(Pos2.getX()-0.5, Pos2.getY()+0.5, 0); // Back Up from mid X
                    Pos4 = VecPosition(Pos2.getX()-0.5, Pos2.getY()-0.5, 0); // Back DOwn from mid X


                    ///play downwards
                    Pos6 =  VecPosition(0, -6, 0);
                    ///play upwards
                    Pos5 =  VecPosition(-3, 6, 0);
                    Pos7 = VecPosition(5,0,0);

                    Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos10 = VecPosition(Pos8.getX()+0.5, Pos8.getY()+2, 0);
                    Pos11 = VecPosition(Pos8.getX()+0.5, Pos8.getY()-2, 0);
                }


            }
            else{

                offset = 0.2;
                offp = ballp.getPointOnLineFraction(orgoal,offset);
                offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                Pos2 = offsetPos; // Middle of X formation of defenders
                Pos3 = VecPosition(Pos2.getX()-1, Pos2.getY()+1, 0); // Back Up from mid X
                Pos5 = VecPosition(Pos2.getX()+1, Pos2.getY()+1, 0); // Forward Up from mid X

                Pos4 = VecPosition(0, -5, 0);
                Pos6 = VecPosition(5,-8,0);
                Pos7 = VecPosition(7,-4,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(-5, -5, 0);
                Pos11 = VecPosition(-5, 0, 0);
            }
        }
        else if(regionX ==2){

            offset = 0.1;
            offp = ballp.getPointOnLineFraction(orgoal,offset);
            offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

            Pos2 = offsetPos; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X

            float dist = VecPosition(0,0,0).getDistanceTo(ball);
            if(dist < 0.2){
                Pos5 = VecPosition(5, 8, 0);   
                Pos6 = VecPosition(0, 5, 0); 
                Pos7 = VecPosition(8, 3,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(Pos2.getX()-3, Pos2.getY()-7, 0);
                Pos11 = VecPosition(Pos2.getX()+3, Pos2.getY()-3, 0);
            }
            else{
                if(ball.getY() > 0){
                Pos5 = VecPosition(5, 8, 0);   
                Pos6 = VecPosition(0, 5, 0); 
                Pos7 = VecPosition(8, 3,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(Pos2.getX()-3, Pos2.getY()-7, 0);
                Pos11 = VecPosition(Pos2.getX()+3, Pos2.getY()-3, 0);

                }
                else{
                    Pos5 = VecPosition(5, -8, 0);
                    Pos6 = VecPosition(0, -5, 0); 
                    Pos7 = VecPosition(8, -3,0);

                    Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos10 = VecPosition(Pos2.getX()-3, Pos2.getY()+7, 0);
                    Pos11 = VecPosition(Pos2.getX()+3, Pos2.getY()+3, 0);

                }
            }

            
        }
        else if(regionX ==3){

            offset = 0.2;
            offp = ballp.getPointOnLineFraction(orgoal,offset);
            offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

            Pos2 = offsetPos; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X


            if(ball.getY() > 0){
                Pos5 = VecPosition(5, 5, 0);
                Pos6 = VecPosition(-2, -7, 0);
                Pos7 = VecPosition(10,3,0);

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(ball.getX()+1, ball.getY()-3, 0);
                Pos11 = VecPosition(7, -1, 0);
            }
            else{
                Pos5 = VecPosition(5, -5, 0);
                Pos6 = VecPosition(-2, 7, 0);
                Pos7 = VecPosition(10,-3,0);

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(ball.getX()+1, ball.getY()+3, 0);
                Pos11 = VecPosition(7, +1, 0);
            }
        }
        else{

            offset = 0.1;
            offp = ballp.getPointOnLineFraction(orgoal,offset);
            offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

            Pos2 = offsetPos; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X

            Pos5 = VecPosition(9, 0, 0);
            Pos6 = VecPosition(12, 2, 0);
            Pos7 = VecPosition(12,-2,0);

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = VecPosition(0, 0, 0);
            if(regionY ==1){
                Pos10 = VecPosition(5, 7, 0);
                Pos11 = VecPosition(5, -5, 0);
            }
            else if(regionY ==2){
                Pos10 = VecPosition(ball.getX()-5, 5, 0);
                Pos11 = VecPosition(ball.getX()-5, -5, 0);
            }
            else{
                Pos10 = VecPosition(5, -7, 0);
                Pos11 = VecPosition(5, 5, 0);
            }
            

        }




    }

    

    

    //Pos1 = ValidateFieldPos(Pos1);
    Pos2 = ValidateFieldPos(Pos2);
    Pos3 = ValidateFieldPos(Pos3);
    Pos4 = ValidateFieldPos(Pos4);
    Pos5 = ValidateFieldPos(Pos5);
    Pos6 = ValidateFieldPos(Pos6);
    Pos7 = ValidateFieldPos(Pos7);
    Pos8 = ValidateFieldPos(Pos8);
    Pos9 = ValidateFieldPos(Pos9);
    Pos10 = ValidateFieldPos(Pos10);
    Pos11 = ValidateFieldPos(Pos11);

    if(visualise){
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos1.getX(), Pos1.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos2.getX(), Pos2.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos3.getX(), Pos3.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos4.getX(), Pos4.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos5.getX(), Pos5.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos6.getX(), Pos6.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos7.getX(), Pos7.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos8.getX(), Pos8.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos9.getX(), Pos9.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos10.getX(), Pos10.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos11.getX(), Pos11.getY(),0.2,1,1,1); //WHITE 
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,offsetPos.getX(),offsetPos.getY(),0.2,1,0,1); //PINK
    }

    ImportantPositions.push_back(Pos1);
    ImportantPositions.push_back(Pos2);
    ImportantPositions.push_back(Pos3);
    ImportantPositions.push_back(Pos4);
    ImportantPositions.push_back(Pos5);
    ImportantPositions.push_back(Pos6);
    ImportantPositions.push_back(Pos7);
    ImportantPositions.push_back(Pos8);
    ImportantPositions.push_back(Pos9);
    ImportantPositions.push_back(Pos10);
    ImportantPositions.push_back(Pos11);

    

    return ImportantPositions;
}


std::vector<double> NaoBehavior::get_joints(){
    std::vector<double> A;
    StateDict stateDict = getCurrentStateDictionaryRL(false);

    // A.push_back(stateDict["ballposrelx"]);
    // A.push_back(stateDict["ballposrely"]);
    // A.push_back(stateDict["ballposrelz"]);
    A.push_back(stateDict["LElbowYaw"]);
    A.push_back(stateDict["LElbowRoll"]);
    A.push_back(stateDict["LAnklePitch"]);
    A.push_back(stateDict["LAnkleRoll"]);
    A.push_back(stateDict["LHipPitch"]);
    A.push_back(stateDict["LHipRoll"]);
    A.push_back(stateDict["LHipYawPitch"]);
    A.push_back(stateDict["LKneePitch"]);
    A.push_back(stateDict["LShoulderPitch"]);
    A.push_back(stateDict["LShoulderRoll"]);
    A.push_back(stateDict["HeadPitch"]);
    A.push_back(stateDict["HeadYaw"]);
    A.push_back(stateDict["RElbowYaw"]);
    A.push_back(stateDict["RElbowRoll"]);
    A.push_back(stateDict["RAnklePitch"]);
    A.push_back(stateDict["RAnkleRoll"]);
    A.push_back(stateDict["RHipPitch"]);
    A.push_back(stateDict["RHipRoll"]);
    A.push_back(stateDict["RHipYawPitch"]);
    A.push_back(stateDict["RKneePitch"]);
    A.push_back(stateDict["RShoulderPitch"]);
    A.push_back(stateDict["RShoulderRoll"]);
    return A;
}
double NaoBehavior::norm(std::vector<double> A){
    double x = 0;
    for (double v: A) x += v * v;
    double ans  = sqrt(x);
    // printf("NORM: %lf\n", ans);
    return ans;
}


// to delete
// mamadou
/* void dummy()
{
    worldModel->getRVSender()->clear();

    int _playerNumber = worldModel->getUNum();
    int regionX = getXFieldRegion(ball);
    int regionY = getYFieldRegion(ball);
    int _playMode = worldModel->getPlayMode();
    int _side = worldModel->getSide();

    VecPosition _myPos = worldModel->getMyPosition();
    _myPos.setZ(0);

    vector<pair<double,int > > TeamDistToBall = GenerateTeamToTargetDistanceVector(_playerNumber, ball);

    vector<VecPosition> ImportantPositions = GenerateImportantPositions(_playMode, _side, _playerNumber, regionX, regionY);
    vector<vector<pair<double,int > > > PreferenceToPointArray = GeneratePreferenceArrayForTeam(_playerNumber,ImportantPositions);
    vector<int> PointPreferences = stableMarriage(PreferenceToPointArray);
    int mydesiredpositionIndex = PointPreferences[_playerNumber-1];
    VecPosition mydesiredposition = ImportantPositions[mydesiredpositionIndex];


    //worldModel->getRVSender()->drawLine("TARGET Traj",_myPos.getX(), _myPos.getY(),mydesiredposition.getX(), mydesiredposition.getY(),RVSender::YELLOW); //YELLOW
   // worldModel->getRVSender()->drawCircle("DESIRED",mydesiredposition.getX(), mydesiredposition.getY(),0.1,0,1,0); //GREEN

    if(isBetweenTargetAndBall(VecPosition(HALF_FIELD_X,0,0),_myPos,0.1)){
        //worldModel->getRVSender()->drawCircle("BehindMeBadFORMATION",_myPos.getX(), _myPos.getY(),0.3,1,0,1); //PINK
        //worldModel->getRVSender()->drawLine("GOAL Traj",ball.getX(), ball.getY(),HALF_FIELD_X, 0,1,0,1); //PINK
        SIM::Point2D origin = SIM::Point2D(_myPos.getX(),_myPos.getY());
        SIM::Point2D adjustedPos = origin.findShortestPositionNearBall(origin,SIM::Point2D(ball.getX(),ball.getY()),SIM::Point2D(mydesiredposition.getX(), mydesiredposition.getY()),0.3);
        //worldModel->getRVSender()->drawCircle("AdjsutementBehindMeBadFORMATION",adjustedPos.x, adjustedPos.y,0.1,0,0,0); 
        //worldModel->getRVSender()->drawLine("GOAL Traj",mydesiredposition.getX(), mydesiredposition.getY(),adjustedPos.x, adjustedPos.y,0,0,0); 

                
    }
    if(AmITheNthClosestTeammateToBall(TeamDistToBall,_playerNumber,0)){
        VecPosition goodlocation =  DetermineGoodKickToLocation(_playerNumber, _myPos);
        worldModel->getRVSender()->drawCircle("AdjsutementBehindMeBadFORMATION",goodlocation.getX(), goodlocation.getY(),0.1,0,0,0); 
        worldModel->getRVSender()->drawLine("GOAL Traj",_myPos.getX(), _myPos.getY(),goodlocation.getX(), goodlocation.getY(),0,0,0); 
    }
    

} */


void NaoBehavior::InitPlayer()
{

    int _playerNumber = worldModel->getUNum();

    if (m_initialized == false)
    {
        if (_playerNumber == 2)
        {
            m_is_passer = true;
            m_is_receiver = false;
        }
           
        if (_playerNumber == 3)
        {
            m_is_passer = false;
            m_is_receiver = true;

        }
          m_initialized = true;
    }


    if (m_is_passer)
    {
        if (m_initial_ball_pos.getX() == -1 )
        {  
            m_initial_ball_pos = VecPosition(ball.getX(),ball.getY(),ball.getZ());

        }
    }

    if (m_is_receiver)
    {

    }
}


void NaoBehavior::OnPassingBall(int pNum,int teamNum,VecPosition myPos,VecPosition teamPos )
{
    float dist_ball = myPos.getDistanceTo(ball);


    if (m_is_passer == false)
      return;
    
     if (  dist_ball <= 0.5 && m_initial_ball_pos.getX() == -1  )
     {
        m_initial_ball_pos = VecPosition(ball.getX(),ball.getY(),ball.getZ());
     }   
    
     if (HasMadeAPass(m_initial_ball_pos))
     {
        m_has_made_pass = true;

        m_initial_ball_pos = VecPosition(-1,-1,-1);

        //m_is_passer = false;
        //m_is_receiver = true;
     }

     if (m_has_made_pass == true)
     {
        // Wait until the ball stop
        
        float dist = teamPos.getDistanceTo(ball);
        if (m_initial_ball_pos.getX() == -1 && dist<=0.5  )
        {
           m_initial_ball_pos = VecPosition(ball.getX(),ball.getY(),ball.getZ());

           m_has_made_pass = false;
        }
        
     }

}

void NaoBehavior::OnReceivingBall(int pNum,int teamNum,VecPosition myPos,VecPosition teamPos)
{
    float dist_ball = myPos.getDistanceTo(ball);


    // Two strategy

    // 1st strategy
    // Move the receiver to the ball has soon as it 
    // has been released from the passer

    // 2nd 
    // Wait until the ball stop
    // then move receiver to the ball


    if ( HasMadeAPass(m_initial_ball_pos))
    {


        m_is_passer = true;
        m_is_receiver = false;

        // Now we must determine zhere the ball stop

        // 
        m_initial_ball_pos = VecPosition(-1,-1,-1);     

     }


    // This is approximately where the ball stop
    /*if (dist_ball <=0.5)
    {
        m_initial_ball_pos = VecPosition(ball.getX(),ball.getY(),ball.getZ());
    } */  

}

void NaoBehavior::OnWaitingTeamToReceive(int pNum,int teamNum,VecPosition myPos,VecPosition teamPos)
{
    
}

bool NaoBehavior::DidBallStop()
{
    VecPosition _myPos = worldModel->getMyPosition();
    float dist_ball = _myPos.getDistanceTo(ball);

     // This is approximately where the ball stop
    if (dist_ball <=0.5 && m_initial_ball_pos.getX() == -1) 
    {
        return true;
    }   


   return false;   
}
// new version v5
SkillType NaoBehavior::selectSkill()
{
    int _playerNumber = worldModel->getUNum();
    VecPosition _myPos = worldModel->getMyPosition();
    _myPos.setZ(0);
    float dist_ball = _myPos.getDistanceTo(ball);
    int id_teammate = - 1;

     // the goal has not to do something
    if (_playerNumber == 1 )
      return SKILL_STAND;
    
    if (_playerNumber == 2)
        id_teammate = 3;

    if (_playerNumber == 3 )
        id_teammate = 2;

    
    VecPosition pos_teammate =  worldModel->getWorldObject(id_teammate)->pos;

    InitPlayer();

    if (m_is_passer)
    {
       OnPassingBall(_playerNumber,id_teammate,_myPos,pos_teammate);

       if (m_has_made_pass == false)
       return kickBall(KICK_FORWARD, pos_teammate);    
    }

    if (m_is_receiver)
    {
       OnReceivingBall(_playerNumber,id_teammate,_myPos,pos_teammate);
    }

    return SKILL_STAND;
}

// v4
// SkillType NaoBehavior::selectSkill() 
// {

//      int _playerNumber = worldModel->getUNum();
//     VecPosition _myPos = worldModel->getMyPosition();
//     _myPos.setZ(0);
//     float dist_ball = _myPos.getDistanceTo(ball);

//       int id_teammate = - 1;

//     if (_playerNumber == 2)
//         id_teammate = 3;

//     if (_playerNumber == 3 )VecPosition(ball.getX(),ball.getY(),ball.getZ());
//         id_teammate = 2;


//     if (_playerNumber == 1 )
//         return SKILL_STAND;

//     VecPosition pos_teammate =  worldModel->getWorldObject(id_teammate)->pos;

//      if (_playerNumber == 2)
//      {
//         //printf("ball = %f  prev_dist  = %f \n",dist_ball,prev_dist);
//         //printf("has pass ? =  %d  \n ",m_has_made_pass);
//      }


//     if (m_initialized == false)
//     {
//         if (_playerNumber == 2)
//         {
//             m_is_passer = true;
//             m_is_receiver = false;
//         }
           
//         if (_playerNumber == 3)
//         {
//             m_is_passer = false;
//             m_is_receiver = true;

//         }
//           m_initialized = true;
//     }

//     double game_time = worldModel->getGameTime();
//     //printf("gametime = %f \n",game_time );

//     if (m_is_passer)
//     {
//       //  printf("I am passer num = %d \n ",_playerNumber);
//     }

//     if (m_is_receiver)
//     {
//         //printf("I am receiver num = %d  \n ",_playerNumber);
//     }

//   // For the player zho is not ther first to made a pass
//   // We need to get the position from whicvh the ball stopped
//    if (m_initial_ball_pos.getX() == -1  && dist_ball < 6)
//    {
//       m_initial_ball_pos =  VecPosition(ball.getX(),ball.getY(),ball.getZ());
     
//       if (m_is_receiver )
//       {

//          double d = m_prev_ball_pos.getDistanceTo(ball);

//         if (_playerNumber == 3)
//          printf("d =  %f \n",d);


//         // Okay the ball did not stop stopped
//         // So the player has to wait 
//           if (d > 0.001f )
//           {
//               m_initial_ball_pos = VecPosition(-1,-1,-1);
//           }
               
//       }
//    }
    
//     m_prev_ball_pos = VecPosition(ball.getX(),ball.getY(),ball.getZ());

//     if (HasMadeAPass(m_initial_ball_pos))
//     {
//         m_has_made_pass = true;
//     }

//     if ( m_is_passer && m_is_receiver == false &&  m_has_made_pass == true)
//     {
//         m_is_passer = false;
//         m_is_receiver = true;

//         return SKILL_STAND;
//      }

//      // Only act if the player position is close to ball
//     if (dist_ball < 6 && m_has_made_pass == false )
//     {  
//         if (m_is_passer)
//           return kickBall(KICK_FORWARD, pos_teammate);           
//     }

//     if (m_is_receiver)
//         return SKILL_STAND;


//     return SKILL_STAND;
// }

/*bool NaoBehaviour::HasMadeAPass()
{
  return false;
}*/

// v3
// SkillType NaoBehavior::selectSkill() 
// {

//      int _playerNumber = worldModel->getUNum();
//     VecPosition _myPos = worldModel->getMyPosition();
//     _myPos.setZ(0);
//     float dist_ball = _myPos.getDistanceTo(ball);

//       int id_teammate = - 1;

//         if (_playerNumber == 2)
//             id_teammate = 3;

//         if (_playerNumber == 3 )
//            id_teammate = 2;

//     VecPosition pos_teammate =  worldModel->getWorldObject(id_teammate)->pos;

//      if (_playerNumber == 2)
//      {
//         //printf("ball = %f  prev_dist  = %f \n",dist_ball,prev_dist);
//         //printf("has pass ? =  %d  \n ",m_has_made_pass);

//      }

//      if (m_has_made_pass == true)
//      {

//         double t1 =  m_timer_recei_ball - worldModel->getGameTime();
        
//         //if ( _playerNumber == 2)
//           //  printf(" receive ball = %d   t1 = %f \n ",m_teammate_has_recei_ball,t1);

//          if ( m_teammate_has_recei_ball == false  &&  t1 <= 0 )
//          {
//                m_teammate_has_recei_ball = true;

//               //  We also compute the timer for llocal player receiving the ball

//               // in experimentation when the distance 
//               float time_to_rece =  _myPos.getDistanceTo(pos_teammate) * 2;

//              //m_timer_recei_ball =  worldModel->getGameTime() + 8;
//               m_timer_recei_ball =  worldModel->getGameTime() + time_to_rece;
//                // Get the position from where the team mate gets the ball
//          }

//         // We suppose we receive the ball noz
//         // bug here
//         // should make team mate received ball to false
//         // because it will go to the first if 
//         if ( m_teammate_has_recei_ball == true && m_timer_recei_ball - worldModel->getGameTime() <= 0)
//         {
//             printf("I am  supposed to receive the ball \n");
//             m_has_made_pass = false;
//             m_teammate_has_recei_ball = false;

//         }
        
//         return SKILL_STAND;

//      }


//      // Only act if the player position is close to ball
//     if (dist_ball < 6 && m_has_made_pass == false )
//     {
      
//         printf("MY ID = %d  I have'nt made a pass yet \n",_playerNumber);
//         //VecPosition pos_teammate =  worldModel->getWorldObject(id_teammate)->pos;

//         if (m_timer_kick_on == false)
//         {
//             m_timer_kick_on = true;

//               // in experimentation when the distance 

//              float dist_ball =   _myPos.getDistanceTo(ball);
//             float time_to_rece =  dist_ball * 4;
        
//             printf("MY ID = %d \n",_playerNumber );
//             printf("Dist ball %f \n",dist_ball);
//             printf(" timer to receive =  %f \n",time_to_rece);

//             m_timer_to_kick =  worldModel->getGameTime() + time_to_rece;

//             //m_timer_to_kick =  worldModel->getGameTime() + 14;
//             //return kickBall(KICK_FORWARD, pos_teammate);
//         }

//         else
//         {
//             float  t_kick = m_timer_to_kick -  worldModel->getGameTime();
//             printf("Timer to kick %f",t_kick);
//                 // We suppose we made a kick
//             if (  m_timer_to_kick -  worldModel->getGameTime() <= 0)
//             {
//                  float time_to_rece =  _myPos.getDistanceTo(pos_teammate);

//                 //m_timer_recei_ball =  worldModel->getGameTime() + 8;

//                 m_has_made_pass = true;

//                 // update the timer kick on
//                 m_timer_kick_on = false;
//                 printf("I made a pass \n ");
//             }
//         }

//         return kickBall(KICK_FORWARD, pos_teammate);
           
//     }

//     return SKILL_STAND;
// }

// version 2 code
// kind of OK behaviour for passing
// SkillType NaoBehavior::selectSkill() 
// {

//      int _playerNumber = worldModel->getUNum();
//     VecPosition _myPos = worldModel->getMyPosition();
//     _myPos.setZ(0);
//     float dist_ball = _myPos.getDistanceTo(ball);

//      if (_playerNumber == 2)
//      {
//         printf("ball = %f  prev_dist  = %f \n",dist_ball,prev_dist);
//         printf("has pass ? =  %d",m_has_made_pass);

//      }

//      if (m_has_made_pass == true)
//      {
//          if ( m_teammate_has_recei_ball == false  &&  m_timer_recei_ball - worldModel->getGameTime() <= 0 )
//          {
//                m_teammate_has_recei_ball = true;

//              //  We also compute the timer for receiving the ball

//              m_timer_recei_ball =  worldModel->getGameTime() + 8;
//                // Get the position from where the team mate gets the ball


//          }

//         // We suppose we receive the ball noz
//         if ( m_teammate_has_recei_ball == true && m_timer_recei_ball - worldModel->getGameTime() <= 0)
//         {
//             m_has_made_pass = false;
//            m_teammate_has_recei_ball = false;

//         }

//         return SKILL_STAND;

//      }


//      // Only act if the player position is close to ball
//     if (dist_ball < 3.5   && m_has_made_pass == false )
//     {

//         if (_playerNumber == 2)
//          {
//              VecPosition pos_teammate =  worldModel->getWorldObject(3)->pos;
               
            
//             if (m_timer_kick_on == false)
//             {
//                 m_timer_kick_on = true;
//                 m_timer_to_kick =  worldModel->getGameTime() + 13;
//                 //return kickBall(KICK_FORWARD, pos_teammate);
//             }

//             else
//             {

//                 // We suppose we made a kick
//                 if (  m_timer_to_kick -  worldModel->getGameTime() <= 0)
//                 {
//                        m_timer_recei_ball =  worldModel->getGameTime() + 8;
//                        m_has_made_pass = true;
//                 }
//             }

//             // We suppoe that the ball was kicked
//             /*if (  prev_dist!= -1 &&   prev_dist < 0.5   && m_has_made_pass == false)
//             {
//                 // Start the timer
//                 m_timer_recei_ball =  worldModel->getGameTime() + 8;
//                 m_has_made_pass = true;

//                 printf(" I made a pass");
//             }*/

//             return kickBall(KICK_FORWARD, pos_teammate);
//          }

//          else if (_playerNumber == 3  )
//          {
//              VecPosition pos_teammate =  worldModel->getWorldObject(2)->pos;

//              return kickBall(KICK_FORWARD, pos_teammate);
//          }
//     }

//     return SKILL_STAND;

// }

// version 1 code
// SkillType NaoBehavior::selectSkill() {

//     int _playerNumber = worldModel->getUNum();
//     VecPosition temp = worldModel->getMyPosition();


//    //return SKILL_STAND;  // Walk to the ball


//    /*  VecPosition temp = worldModel->getMyPosition();

//     int _playerNumber = worldModel->getUNum(); // returns this agents player number



//     vector<pair<double,int > > TeamDistToBall = GenerateTeamToTargetDistanceVector(_playerNumber, ball);

//     bool closest = AmITheNthClosestTeammateToBall(TeamDistToBall,_playerNumber,0);


//     float dist_ball = temp.getDistanceTo(ball);
     

//     if ( _playerNumber ==  player_possess_ball )
//     {
//         return SKILL_STAND;
//     }

//     else
//     {
//        if (_playerNumber == 2)
//          {
//              VecPosition pos_teammate =  worldModel->getWorldObject(3)->pos;

//              return kickBall(KICK_FORWARD, pos_teammate);
//          }

//          else if (_playerNumber == 3  )
//          {
//              VecPosition pos_teammate =  worldModel->getWorldObject(2)->pos;

//              return kickBall(KICK_FORWARD, pos_teammate);
//          }
//     }
//  */

//      // passing ball each other

//     float dist_ball = temp.getDistanceTo(ball);
    
//     bool is_ball_kicked = false;

//     if (dist_ball > 4)
//     {
//          //printf(" pNum =   %d  ball far away \n ",_playerNumber);
//        return SKILL_STAND;
//     }

//     else
//     {

//         printf(" pNum = %d possess ball %d \n ",_playerNumber,player_possess_ball);

//         // Check who is in possess

//          if (_playerNumber == 2)
//          {
//              VecPosition pos_teammate = worldModel->getTeammate(3);           
//             float dist = pos_teammate.getDistanceTo(ball) ;
//             printf(" pNum = %d dist_teammate_ball  %f \n ",_playerNumber,dist);

//              if (dist < 0.6f  )
//                  player_possess_ball = 3;
//          }


//          if (_playerNumber == 3)
//          {
//              VecPosition pos_teammate = worldModel->getTeammate(2);

//              float dist = pos_teammate.getDistanceTo(ball) ;

//               printf(" pNum = %d dist_teammate_ball  %f \n ",_playerNumber,dist);

//              if (dist  < 0.6f  )
//                  player_possess_ball = 2;
//          }


//         if (player_possess_ball == _playerNumber)
//            return SKILL_STAND;

//         //printf(" pNum =   %d  I am thinking \n ",_playerNumber);
//         float dist_kick = 0.5; 
       
//         // We suppose he made a kick
//         if (  abs(dist_ball - prev_dist ) > 1 && prev_dist != -1  )
//         {
//              is_ball_kicked = true;
//             player_possess_ball =    _playerNumber;

//             printf(" pNum =   %d  I am kicking \n ",_playerNumber);
//         }

//         // check team
//         if (_playerNumber == 2)
//          {
            
//              printf(" pNum =   %d  I am going to kick \n ",_playerNumber);
//              VecPosition pos_teammate =  worldModel->getWorldObject(3)->pos;

//              //printf("dist %f ",dist_ball);

//              return kickBall(KICK_FORWARD, pos_teammate);
//          }

//          else if (_playerNumber == 3  )
//          {
//              printf(" pNum =   %d  I am going to kick \n ",_playerNumber);
//            // player_possess_ball = 3;
//              VecPosition pos_teammate =  worldModel->getWorldObject(2)->pos;

//              return kickBall(KICK_FORWARD, pos_teammate);
//          }

//     }

    
//     //printf(" player_num = %d  closest ? =  %d ",_playerNumber,closest);

//     // the closesrt player make pass
//     /*if(AmITheNthClosestTeammateToBall(TeamDistToBall,_playerNumber,0))
//     {


//         // go make a pass to team mate
//          if (_playerNumber == 2)
//          {
//              VecPosition pos_teammate =  worldModel->getWorldObject(3)->pos;

//              return kickBall(KICK_FORWARD, pos_teammate);
//          }

//          else if (_playerNumber == 3  )
//          {
//              VecPosition pos_teammate =  worldModel->getWorldObject(2)->pos;

//              return kickBall(KICK_FORWARD, pos_teammate);
//          }
//     }

//     else
//     {
//         return SKILL_STAND;
//     }*/
    

//     // if(_playerNumber ==2){

//     //   VecPosition temp = worldModel->getMyPosition();
   
//     //   float distance = temp.getDistanceTo(ball);

//     //  //printf("distance =  %f ",distance);

//     //   //if (distance < 1) 
//     //     // return SKILL_STAND;

//     //    VecPosition pos_teammate =  worldModel->getWorldObject(3)->pos;

//     //    return kickBall(KICK_FORWARD, pos_teammate);
        
//     //     //return kickBall(KICK_FORWARD, VecPosition(10, 0, 0)); // Basic kick
//     // }
//     // else{
//     //     return SKILL_STAND;
//     // }

//     // if (_playerNumber == 3)
//     // {

//     // }


//     //  Mamadou Code
//       //VecPosition temp = worldModel->getMyPosition();
   

//       //float distance = temp.getDistanceTo(ball);

//       //if (distance < 0.1f) 
//         // return SKILL_STAND;
        
//     //VecPosition pos_ball =  ball.Midpoint();

//     #ifdef IS_MEASURING_KICKS
//         std::string curr_kick_to_use = mike_measure_kick_types_to_run[mike_measure_current_kick_idx];
//         if (skill == SKILL_STAND){
//             // std::cout << "DONE HERE. Ball = " << worldModel->getW
//         }
//         return doRLKick(VecPosition(HALF_FIELD_X, -HALF_FIELD_Y, 0), curr_kick_to_use);
//     #endif
//     // return doRLKick(VecPosition(HALF_FIELD_X, -HALF_FIELD_Y, 0), "long");
//     // return doRLKick(VecPosition(HALF_FIELD_X, 0, 0), "long");
//     // return doRLKick(VecPosition(HALF_FIELD_X, 0, 0), "normal");
//     // return doRLKick(VecPosition(HALF_FIELD_X, -HALF_FIELD_Y, 0), "normal");

//     std::vector<std::pair<double,int>> X;
//     // return DetermineAppropriateRLKick(me, X, VecPosition(HALF_FIELD_X, -HALF_FIELD_Y, 0));
//     // return doRLKick(VecPosition(HALF_FIELD_X, -HALF_FIELD_Y, 0));
//     // print_state();
//     // return SKILL_STAND;
//     // return doRLKick(VecPosition(HALF_FIELD_X, 0, 0));

//      _playerNumber = worldModel->getUNum();
//     VecPosition _myPos = worldModel->getMyPosition();
//     _myPos.setZ(0);

//     // return WitsGoToFormationBehaviour();

//     // if (fFatProxy){
//     //     return WitsFATDecisionTreeBehaviour();
//     // }
//     // else{
//     //     return WitsDecisionTreeBehaviour();
//     //     return WitsDecisionTreeBehaviour2023();
//     // }
// }


vector<VecPosition> NaoBehavior::GenerateImportantPositionsOld(int _playMode, int _side, int _playerNumber, int regionX, int regionY){

    vector<VecPosition> ImportantPositions;
    std::string SpNum = std::to_string(_playerNumber);
    double offset = 0;
    SIM::Point2D ballp = SIM::Point2D(ball.getX(), ball.getY());
    SIM::Point2D orgoal = SIM::Point2D(-HALF_FIELD_X,0);
    SIM::Point2D offp = ballp.getPointOnLineFraction(orgoal,offset);
    VecPosition offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 
    VecPosition MidBoalGoal = ball.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));

    VecPosition Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
    VecPosition Pos2 = ball.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0)); // Middle of X formation of defenders
    VecPosition Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
    VecPosition Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
    VecPosition Pos5 = VecPosition(Pos2.getX()+2, Pos2.getY()+2, 0); // Forward Up from mid X
    VecPosition Pos6 = VecPosition(Pos2.getX()+2, Pos2.getY()-2, 0); // Forward DOwn from mid X
    VecPosition Pos7 = VecPosition(ball.getX()+ 3, ball.getY(),0); // Infront of ball

    VecPosition Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
    VecPosition Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
    VecPosition Pos10 = ball.Midpoint(Pos2);
    Pos10.setY(Pos10.getY()+1);
    VecPosition Pos11 = ball.Midpoint(Pos2);
    Pos11.setY(Pos11.getY()-1);

    
    if(_playMode == PM_BEFORE_KICK_OFF){
        Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
        Pos2 = MidBoalGoal; // Middle of X formation of defenders
        Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
        Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
        Pos5 = VecPosition(ball.getX()-0.3, ball.getY()+3, 0); // Forward Up from ball
        Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-3, 0); // Forward DOwn from ball
        Pos7 = VecPosition(ball.getX()-4, ball.getY(),0); // Behind ball

        Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
        Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
        Pos10 = ball.Midpoint(Pos2);
        Pos10.setY(Pos10.getY()+1);
        Pos11 = ball.Midpoint(Pos2);
        Pos11.setY(Pos11.getY()-1);


    }
    else if(_playMode == PM_KICK_OFF_LEFT){
        if(_side == SIDE_LEFT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-2, ball.getY()+2, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-2, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-4, ball.getY(),0); // On the ball

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = VecPosition(ball.getX()-3, ball.getY()+4, 0); // Forward Up from ball
            Pos11 = VecPosition(ball.getX()-5, ball.getY()+8, 0); // Forward Up from ball
        }
        else{
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-0.3, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = ball.Midpoint(Pos2);
            Pos10.setY(Pos10.getY()+5);
            Pos11 = ball.Midpoint(Pos2);
            Pos11.setY(Pos11.getY()-5);
        }
    }
    else if(_playMode == PM_KICK_OFF_RIGHT){
        if(_side== SIDE_RIGHT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-2, ball.getY()+2, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-2, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-4.5, ball.getY(),0); // On the ball

            Pos8 = VecPosition(5, 0, 0);
            Pos9 = VecPosition(-HALF_FIELD_X+5, 0, 0);
            Pos10 = VecPosition(-HALF_FIELD_X+2, 2.5, 0);
            Pos11 = VecPosition(-HALF_FIELD_X+2, -2.5, 0);
            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = VecPosition(ball.getX()-3, ball.getY()+4, 0); // Forward Up from ball
            Pos11 = VecPosition(ball.getX()-5, ball.getY()+8, 0); // Forward Up from ball


        }
        else{
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-0.3, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-0.3, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos10 = ball.Midpoint(Pos2);
            Pos10.setY(Pos10.getY()+5);
            Pos11 = ball.Midpoint(Pos2);
            Pos11.setY(Pos11.getY()-5);
        }
    }
    
    else if((_playMode == PM_FREE_KICK_LEFT) || (_playMode == PM_KICK_IN_LEFT) || (_playMode == PM_GOAL_KICK_LEFT)){
        if(_side == SIDE_LEFT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.2,0,0); // Our Goal
            Pos2 = VecPosition(-HALF_FIELD_X/2,0,0); // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos6 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos7 = VecPosition(Pos2.getX()+5, ball.getY(),0); // On ball
            Pos8 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos9 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos10 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos11 = VecPosition(-HALF_FIELD_X+3,0,0); // On ball
        }
        else {
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-2, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-2, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball
        }
    }
    else if((_playMode == PM_FREE_KICK_RIGHT) || (_playMode == PM_KICK_IN_RIGHT) || (_playMode == PM_GOAL_KICK_RIGHT)){
        if(worldModel->getSide() == SIDE_RIGHT){
            Pos1 = VecPosition(-HALF_FIELD_X+0.2,0,0); // Our Goal
            Pos2 = VecPosition(-HALF_FIELD_X/2,0,0); // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos6 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos7 = VecPosition(Pos2.getX()+5, ball.getY(),0); // On ball
            Pos8 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos9 = VecPosition(Pos2.getX()+5, ball.getY()+2, 0); // Up from ball
            Pos10 = VecPosition(Pos2.getX()+5, ball.getY()-2, 0); // Down from ball
            Pos11 = VecPosition(-HALF_FIELD_X+3,0,0); // On ball
        }
        else {
            Pos1 = VecPosition(-HALF_FIELD_X+0.3,0,0); // Our Goal
            Pos2 = MidBoalGoal; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
            Pos5 = VecPosition(ball.getX()-2, ball.getY()+3, 0); // Forward Up from ball
            Pos6 = VecPosition(ball.getX()-2, ball.getY()-3, 0); // Forward DOwn from ball
            Pos7 = VecPosition(ball.getX()-3, ball.getY(),0); // Behind ball
        }
        
    }
    else{ // PM_PLAY_ON 

        double yValue = MapValueInRangeTo(ball.getY(), -HALF_FIELD_Y, HALF_FIELD_Y, -HALF_GOAL_Y, HALF_GOAL_Y);

        Pos1 = VecPosition(-HALF_FIELD_X+0.3,yValue,0); // Our Goal
        Pos2 = MidBoalGoal; // Middle of X formation of defenders
        Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
        Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X
        Pos5 = VecPosition(Pos2.getX()+2, Pos2.getY()+2, 0); // Forward Up from mid X
        Pos6 = VecPosition(Pos2.getX()+2, Pos2.getY()-2, 0); // Forward DOwn from mid X
        Pos7 = VecPosition(ball.getX(), ball.getY(),0); // Infront of ball

        if(regionX ==1){
            if(regionY ==1){

                offset = 0.2;
                offp = ballp.getPointOnLineFraction(orgoal,offset);
                offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                Pos2 = offsetPos; // Middle of X formation of defenders
                Pos4 = VecPosition(Pos2.getX()-1, Pos2.getY()-1, 0); // Back DOwn from mid X
                Pos6 = VecPosition(Pos2.getX()+1, Pos2.getY()-1, 0); // Forward DOwn from mid X

                Pos3 = VecPosition(0, 5, 0);
                Pos5 = VecPosition(5,8,0); 
                Pos7 = VecPosition(-5,9,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(-5, 5, 0);
                Pos11 = VecPosition(-5, 0, 0);
            }
            else if(regionY ==2){

                //this is our middle block region containing our goal

                if(BallinMyGoalMouth()){
                    offset = 0.08;
                    offp = ballp.getPointOnLineFraction(orgoal,offset);
                    offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                    Pos2 = offsetPos; // Middle of X formation of defenders
                    Pos3 = VecPosition(Pos1.getX()+0.2, Pos1.getY()+0.6, 0); // Back Up from mid X
                    Pos4 = VecPosition(Pos1.getX()+0.2, Pos1.getY()-0.6, 0); // Back DOwn from mid X


                    ///play downwards
                    Pos6 =  VecPosition(-5, -8, 0);
                    ///play upwards
                    Pos5 =  VecPosition(-5, 8, 0);
                    Pos7 = VecPosition(-5,0,0);

                    Pos8 = VecPosition(5, 0, 0);
                    Pos9 = VecPosition(-HALF_FIELD_X+5, 0, 0);
                    Pos10 = VecPosition(-HALF_FIELD_X+2, 2.5, 0);
                    Pos11 = VecPosition(-HALF_FIELD_X+2, -2.5, 0);
                }
                else{

                    offset = 0.08;
                    offp = ballp.getPointOnLineFraction(orgoal,offset);
                    offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                    Pos2 = offsetPos; // Middle of X formation of defenders
                    Pos3 = VecPosition(Pos2.getX()-0.5, Pos2.getY()+0.5, 0); // Back Up from mid X
                    Pos4 = VecPosition(Pos2.getX()-0.5, Pos2.getY()-0.5, 0); // Back DOwn from mid X


                    ///play downwards
                    Pos6 =  VecPosition(-5, -8, 0);
                    ///play upwards
                    Pos5 =  VecPosition(-5, 8, 0);
                    Pos7 = VecPosition(-5,0,0);

                    Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos10 = VecPosition(Pos8.getX()+0.5, Pos8.getY()+2, 0);
                    Pos11 = VecPosition(Pos8.getX()+0.5, Pos8.getY()-2, 0);
                }


            }
            else{

                offset = 0.2;
                offp = ballp.getPointOnLineFraction(orgoal,offset);
                offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

                Pos2 = offsetPos; // Middle of X formation of defenders
                Pos3 = VecPosition(Pos2.getX()-1, Pos2.getY()+1, 0); // Back Up from mid X
                Pos5 = VecPosition(Pos2.getX()+1, Pos2.getY()+1, 0); // Forward Up from mid X

                Pos4 = VecPosition(0, -5, 0);
                Pos6 = VecPosition(5,-8,0);
                Pos7 = VecPosition(-5,-9,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(-5, -5, 0);
                Pos11 = VecPosition(-5, 0, 0);
            }
        }
        else if(regionX ==2){

            offset = 0.1;
            offp = ballp.getPointOnLineFraction(orgoal,offset);
            offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

            Pos2 = offsetPos; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X

            float dist = VecPosition(0,0,0).getDistanceTo(ball);
            if(dist < 0.2){
                Pos5 = VecPosition(5, 8, 0);   
                Pos6 = VecPosition(0, 5, 0); 
                Pos7 = VecPosition(3, 3,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(Pos2.getX()-3, Pos2.getY()-7, 0);
                Pos11 = VecPosition(Pos2.getX()+3, Pos2.getY()-3, 0);
            }
            else{
                if(ball.getY() > 0){
                Pos5 = VecPosition(5, 8, 0);   
                Pos6 = VecPosition(0, 5, 0); 
                Pos7 = VecPosition(3, 3,0); 

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(Pos2.getX()-3, Pos2.getY()-7, 0);
                Pos11 = VecPosition(Pos2.getX()+3, Pos2.getY()-3, 0);

                }
                else{
                    Pos5 = VecPosition(5, -8, 0);
                    Pos6 = VecPosition(0, -5, 0); 
                    Pos7 = VecPosition(3, -3,0);

                    Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                    Pos10 = VecPosition(Pos2.getX()-3, Pos2.getY()+7, 0);
                    Pos11 = VecPosition(Pos2.getX()+3, Pos2.getY()+3, 0);

                }
            }

            
        }
        else if(regionX ==3){

            offset = 0.2;
            offp = ballp.getPointOnLineFraction(orgoal,offset);
            offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

            Pos2 = offsetPos; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X


            if(ball.getY() > 0){
                Pos5 = VecPosition(5, 5, 0);
                Pos6 = VecPosition(-2, -7, 0);
                Pos7 = VecPosition(10,3,0);

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(ball.getX()+1, ball.getY()-3, 0);
                Pos11 = VecPosition(7, -1, 0);
            }
            else{
                Pos5 = VecPosition(5, -5, 0);
                Pos6 = VecPosition(-2, 7, 0);
                Pos7 = VecPosition(10,-3,0);

                Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos9 = Pos8.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
                Pos10 = VecPosition(ball.getX()+1, ball.getY()+3, 0);
                Pos11 = VecPosition(7, +1, 0);
            }
        }
        else{

            offset = 0.1;
            offp = ballp.getPointOnLineFraction(orgoal,offset);
            offsetPos = VecPosition(offp.getX(), offp.getY(), 0); 

            Pos2 = offsetPos; // Middle of X formation of defenders
            Pos3 = VecPosition(Pos2.getX()-2, Pos2.getY()+2, 0); // Back Up from mid X
            Pos4 = VecPosition(Pos2.getX()-2, Pos2.getY()-2, 0); // Back DOwn from mid X

            Pos5 = VecPosition(9, 0, 0);
            Pos6 = VecPosition(12, 2, 0);
            Pos7 = VecPosition(12,-2,0);

            Pos8 = Pos2.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            Pos9 = VecPosition(0, 0, 0);
            if(regionY ==1){
                Pos10 = VecPosition(5, 7, 0);
                Pos11 = VecPosition(5, -5, 0);
            }
            else if(regionY ==2){
                Pos10 = VecPosition(ball.getX()-5, 5, 0);
                Pos11 = VecPosition(ball.getX()-5, -5, 0);
            }
            else{
                Pos10 = VecPosition(5, -7, 0);
                Pos11 = VecPosition(5, 5, 0);
            }
            

        }




    }

    

    

    //Pos1 = ValidateFieldPos(Pos1);
    Pos2 = ValidateFieldPos(Pos2);
    Pos3 = ValidateFieldPos(Pos3);
    Pos4 = ValidateFieldPos(Pos4);
    Pos5 = ValidateFieldPos(Pos5);
    Pos6 = ValidateFieldPos(Pos6);
    Pos7 = ValidateFieldPos(Pos7);
    Pos8 = ValidateFieldPos(Pos8);
    Pos9 = ValidateFieldPos(Pos9);
    Pos10 = ValidateFieldPos(Pos10);
    Pos11 = ValidateFieldPos(Pos11);

    if(visualise){
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos1.getX(), Pos1.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos2.getX(), Pos2.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos3.getX(), Pos3.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos4.getX(), Pos4.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos5.getX(), Pos5.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos6.getX(), Pos6.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos7.getX(), Pos7.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos8.getX(), Pos8.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos9.getX(), Pos9.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos10.getX(), Pos10.getY(),0.2,1,1,1); //WHITE
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,Pos11.getX(), Pos11.getY(),0.2,1,1,1); //WHITE 
        worldModel->getRVSender()->drawCircle("Important Position" + SpNum,offsetPos.getX(),offsetPos.getY(),0.2,1,0,1); //PINK
    }

    ImportantPositions.push_back(Pos1);
    ImportantPositions.push_back(Pos2);
    ImportantPositions.push_back(Pos3);
    ImportantPositions.push_back(Pos4);
    ImportantPositions.push_back(Pos5);
    ImportantPositions.push_back(Pos6);
    ImportantPositions.push_back(Pos7);
    ImportantPositions.push_back(Pos8);
    ImportantPositions.push_back(Pos9);
    ImportantPositions.push_back(Pos10);
    ImportantPositions.push_back(Pos11);

    

    return ImportantPositions;
}


/*
 * Demo behavior where players form a rotating circle and kick the ball
 * back and forth
 */
SkillType NaoBehavior::demoKickingCircle() {
    // Parameters for circle
    VecPosition center = VecPosition(-HALF_FIELD_X/2.0, 0, 0);
    double circleRadius = 5.0;
    double rotateRate = 2.5;

    // Find closest player to ball
    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i) {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() == playerNum) {
            // This is us
            temp = worldModel->getMyPosition();
        } else {
            WorldObject* teammate = worldModel->getWorldObject( i );
            if (teammate->validPosition) {
                temp = teammate->pos;
            } else {
                continue;
            }
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall) {
            playerClosestToBall = playerNum;
            closestDistanceToBall = distanceToBall;
        }
    }

    if (playerClosestToBall == worldModel->getUNum()) {
        // Have closest player kick the ball toward the center
        return kickBall(KICK_FORWARD, center);
    } else {
        // Move to circle position around center and face the center
        VecPosition localCenter = worldModel->g2l(center);
        SIM::AngDeg localCenterAngle = atan2Deg(localCenter.getY(), localCenter.getX());

        // Our desired target position on the circle
        // Compute target based on uniform number, rotate rate, and time
        VecPosition target = center + VecPosition(circleRadius,0,0).rotateAboutZ(360.0/(NUM_AGENTS-1)*(worldModel->getUNum()-(worldModel->getUNum() > playerClosestToBall ? 1 : 0)) + worldModel->getTime()*rotateRate);

        // Adjust target to not be too close to teammates or the ball
        target = collisionAvoidance(true /*teammate*/, false/*opponent*/, true/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);

        if (me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
            // Close enough to desired position and orientation so just stand
            return SKILL_STAND;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face center
            return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
    }
}

void NaoBehavior::JSONLOG_StrategyDataToLog(string Actiontype,VecPosition location){
    
    j["Action"]["type"] = Actiontype;
    if (location != NULL){
        j["Action"]["location"] = {location.getX(), location.getY()};
    }
    

}


SkillType NaoBehavior::doSetPlays(int _playMode, int _playerNumber, vector<pair<double,int > > TeamDistToBall, VecPosition mydesiredposition, VecPosition _myPos, vector<int> PointPreferences, vector<VecPosition> ImportantPositions ){

    if(_playMode == PM_KICK_OFF_LEFT){
        if(worldModel->getSide() == SIDE_LEFT){
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                
                return doRLKick(VecPosition(-5, 0, 0)); // mike 2023/07/02
                return kickBall(KICK_FORWARD, VecPosition(-2.5, 4, 0));
            }
        }
        else{
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
    }
    else if(_playMode == PM_KICK_OFF_RIGHT){
        if(worldModel->getSide() == SIDE_RIGHT){
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                return doRLKick(VecPosition(-5, 0, 0)); // mike 2023/07/02
                return kickBall(KICK_FORWARD, VecPosition(-2.5, 4, 0));
            }
        }
        else{
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
    }
    else if(_playMode == PM_FREE_KICK_LEFT || (_playMode == PM_KICK_IN_LEFT) || (_playMode == PM_GOAL_KICK_LEFT)) {
                   // cout << worldModel->getSide() << endl;

        if(worldModel->getSide() == SIDE_LEFT){
                               // cout << "side == right" << endl;
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                    //cout << _playerNumber << "is closest" << endl;
                VecPosition target = getClosestTeammatePos(_playerNumber,TeamDistToBall);
                target = OptimalPassPosition(_playerNumber, VecPosition(0,0,0),PointPreferences,ImportantPositions);
                return doRLKick(target); // mike 2023/07/02
                return kickBall(KICK_IK, target);
            }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            }
        }
        else{
             cout << "side == left" << endl;
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
    }
    else if(_playMode == PM_FREE_KICK_RIGHT || (_playMode == PM_KICK_IN_RIGHT) || (_playMode == PM_GOAL_KICK_RIGHT)){
        if(worldModel->getSide() == SIDE_RIGHT){
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                    //cout << _playerNumber << "is closest" << endl;
                VecPosition target = getClosestTeammatePos(_playerNumber,TeamDistToBall);
                target = OptimalPassPosition(_playerNumber, VecPosition(0,0,0),PointPreferences,ImportantPositions);
                return doRLKick(target); // mike 2023/07/02
                return kickBall(KICK_IK, target);
            }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            }
        }
        else{
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
        
    }
    else if((_playMode == PM_GAME_OVER) || (_playMode == PM_GOAL_LEFT) || (_playMode == PM_GOAL_RIGHT)){
            return SKILL_STAND;
    }
}

SkillType NaoBehavior::WitsGoToFormationBehaviour() {
     worldModel->getRVSender()->clear();

    int _playerNumber = worldModel->getUNum();
    int regionX = getXFieldRegion(ball);
    int regionY = getYFieldRegion(ball);
    int _playMode = worldModel->getPlayMode();
    int _side = worldModel->getSide();

    VecPosition _myPos = worldModel->getMyPosition();
    _myPos.setZ(0);

    vector<pair<double,int > > TeamDistToBall = GenerateTeamToTargetDistanceVector(_playerNumber, ball);

    vector<VecPosition> ImportantPositions = GenerateImportantPositions(_playMode, _side, _playerNumber, regionX, regionY);
    vector<vector<pair<double,int > > > PreferenceToPointArray = GeneratePreferenceArrayForTeam(_playerNumber,ImportantPositions);
    vector<int> PointPreferences = stableMarriage(PreferenceToPointArray);
    int mydesiredpositionIndex = PointPreferences[_playerNumber-1];
    VecPosition mydesiredposition = ImportantPositions[mydesiredpositionIndex];


    //worldModel->getRVSender()->drawLine("TARGET Traj",_myPos.getX(), _myPos.getY(),mydesiredposition.getX(), mydesiredposition.getY(),RVSender::YELLOW); //YELLOW
   // worldModel->getRVSender()->drawCircle("DESIRED",mydesiredposition.getX(), mydesiredposition.getY(),0.1,0,1,0); //GREEN

    if(isBetweenTargetAndBall(VecPosition(HALF_FIELD_X,0,0),_myPos,0.1)){
        //worldModel->getRVSender()->drawCircle("BehindMeBadFORMATION",_myPos.getX(), _myPos.getY(),0.3,1,0,1); //PINK
        //worldModel->getRVSender()->drawLine("GOAL Traj",ball.getX(), ball.getY(),HALF_FIELD_X, 0,1,0,1); //PINK
        SIM::Point2D origin = SIM::Point2D(_myPos.getX(),_myPos.getY());
        SIM::Point2D adjustedPos = origin.findShortestPositionNearBall(origin,SIM::Point2D(ball.getX(),ball.getY()),SIM::Point2D(mydesiredposition.getX(), mydesiredposition.getY()),0.3);
        //worldModel->getRVSender()->drawCircle("AdjsutementBehindMeBadFORMATION",adjustedPos.x, adjustedPos.y,0.1,0,0,0); 
        //worldModel->getRVSender()->drawLine("GOAL Traj",mydesiredposition.getX(), mydesiredposition.getY(),adjustedPos.x, adjustedPos.y,0,0,0); 

                
    }
    if(AmITheNthClosestTeammateToBall(TeamDistToBall,_playerNumber,0)){
        VecPosition goodlocation =  DetermineGoodKickToLocation(_playerNumber, _myPos);
        worldModel->getRVSender()->drawCircle("AdjsutementBehindMeBadFORMATION",goodlocation.getX(), goodlocation.getY(),0.1,0,0,0); 
        worldModel->getRVSender()->drawLine("GOAL Traj",_myPos.getX(), _myPos.getY(),goodlocation.getX(), goodlocation.getY(),0,0,0); 
    }
    

    return SmartGoToTarget(mydesiredposition,1); 

}

SkillType NaoBehavior::WitsFATDecisionTreeBehaviour(){

    int regionX = getXFieldRegion(ball);
    int regionY = getYFieldRegion(ball);
    int _playMode = worldModel->getPlayMode();
    int _side = worldModel->getSide();

    worldModel->getRVSender()->clear();
    int _playerNumber = worldModel->getUNum();
    vector<pair<double,int > > TeamDistToBall = GenerateTeamToTargetDistanceVector(_playerNumber, ball);
    vector<pair<double,int > > TeamDistToOppGoal = GenerateTeamToTargetDistanceVector(_playerNumber, VecPosition(HALF_FIELD_X, 0, 0));
    vector<pair<double,int > > OppDistToBall = GenerateOppToTargetDistanceVector(ball);
    VecPosition target;
    VecPosition _myPos = worldModel->getMyPosition();
    _myPos.setZ(0);


    vector<VecPosition> ImportantPositions = GenerateImportantPositionsFAT(_playMode, _side, _playerNumber, regionX, regionY);
    vector<vector<pair<double,int > > > PreferenceToPointArray = GeneratePreferenceArrayForTeam(_playerNumber,ImportantPositions);
    vector<int> PointPreferences = stableMarriage(PreferenceToPointArray);
    int mydesiredpositionIndex = PointPreferences[_playerNumber-1];
    VecPosition mydesiredposition = ImportantPositions[mydesiredpositionIndex];


    if(_playMode != PM_PLAY_ON){
        if(_playMode == PM_KICK_OFF_LEFT){
            if(worldModel->getSide() == SIDE_LEFT){
                if(isClosestTeam( _playerNumber, TeamDistToBall)){
                    
                    return kickBallFat(ground_pass, VecPosition(-5, 0, 0), mydesiredposition);
                    return doRLKick(VecPosition(-5, 0, 0), "normal"); // mike 2023/07/02
                    return kickBall(KICK_FORWARD, VecPosition(-2.5, 4, 0));
                }
            }
            else{
                float distance = _myPos.getDistanceTo(mydesiredposition);
                if(distance < 0.5){
                        if(visualise){
                            worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                        }
                        return FaceBall(_myPos);
                    }
                else{
                    return SmartGoToTarget(mydesiredposition,0.5);
                } 
            }
        }
        else if(_playMode == PM_KICK_OFF_RIGHT){
            if(worldModel->getSide() == SIDE_RIGHT){
                if(isClosestTeam( _playerNumber, TeamDistToBall)){
                    return kickBallFat(ground_pass, VecPosition(-5, 0, 0), mydesiredposition);
                    return doRLKick(VecPosition(-5, 0, 0), "normal"); // mike 2023/07/02
                    return kickBall(KICK_FORWARD, VecPosition(-2.5, 4, 0));
                }
            }
            else{
                float distance = _myPos.getDistanceTo(mydesiredposition);
                if(distance < 0.5){
                        if(visualise){
                            worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                        }
                        return FaceBall(_myPos);
                    }
                else{
                    return SmartGoToTarget(mydesiredposition,0.5);
                } 
            }
        }
        else if(_playMode == PM_FREE_KICK_LEFT || (_playMode == PM_KICK_IN_LEFT) || (_playMode == PM_GOAL_KICK_LEFT)) {
                    // cout << worldModel->getSide() << endl;

            if(worldModel->getSide() == SIDE_LEFT){
                                // cout << "side == right" << endl;
                if(isClosestTeam( _playerNumber, TeamDistToBall)){
                        //cout << _playerNumber << "is closest" << endl;
                    VecPosition target = getClosestTeammatePos(_playerNumber,TeamDistToBall);
                    target = OptimalPassPosition(_playerNumber, VecPosition(0,0,0),PointPreferences,ImportantPositions);
                    return kickBallFat(ground_pass, target, mydesiredposition);
                    return doRLKick(target, "long"); // mike 2023/07/02
                    return kickBall(KICK_IK, target);
                }
                else{
                    return SmartGoToTarget(mydesiredposition,0.5);
                }
            }
            else{
                cout << "side == left" << endl;
                float distance = _myPos.getDistanceTo(mydesiredposition);
                if(distance < 0.5){
                        if(visualise){
                            worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                        }
                        return FaceBall(_myPos);
                    }
                else{
                    return SmartGoToTarget(mydesiredposition,0.5);
                } 
            }
        }
        else if(_playMode == PM_FREE_KICK_RIGHT || (_playMode == PM_KICK_IN_RIGHT) || (_playMode == PM_GOAL_KICK_RIGHT)){
            if(worldModel->getSide() == SIDE_RIGHT){
                if(isClosestTeam( _playerNumber, TeamDistToBall)){
                        //cout << _playerNumber << "is closest" << endl;
                    VecPosition target = getClosestTeammatePos(_playerNumber,TeamDistToBall);
                    target = OptimalPassPosition(_playerNumber, VecPosition(0,0,0),PointPreferences,ImportantPositions);
                    return kickBallFat(ground_pass, target, mydesiredposition);
                    return doRLKick(target, "long");
                    return kickBall(KICK_IK, target);
                }
                else{
                    return SmartGoToTarget(mydesiredposition,0.5);
                }
            }
            else{
                float distance = _myPos.getDistanceTo(mydesiredposition);
                if(distance < 0.5){
                        if(visualise){
                            worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                        }
                        return FaceBall(_myPos);
                    }
                else{
                    return SmartGoToTarget(mydesiredposition,0.5);
                } 
            }
            
        }
        else if((_playMode == PM_GAME_OVER) || (_playMode == PM_GOAL_LEFT) || (_playMode == PM_GOAL_RIGHT)){
                return SKILL_STAND;
        }
    }
    


    //if(isOffensiveStrat(_playerNumber,TeamDistToBall,OppDistToBall)){
        if(isClosestTeam( _playerNumber, TeamDistToBall)){

            if(isShootingRange(_myPos, FAT_SHOOTING_RANGE)){

                target = DetermineShootAtGoalPosition(_myPos, _playerNumber, TeamDistToBall); 
                worldModel->getRVSender()->drawPoint("shootball", target.getX(), target.getY(), 10.0f, RVSender::MAGENTA);      


                if(isBetweenTargetAndBall(target,_myPos,0.3)){ //make sure we are not self kicking
                    worldModel->getRVSender()->drawCircle("BehindMeBadBAD",_myPos.getX(), _myPos.getY(),0.4,1,0,1); //PINK
                    SIM::Point2D origin = SIM::Point2D(_myPos.getX(),_myPos.getY());
                    SIM::Point2D adjustedPos = origin.findShortestPositionNearBall(origin,SIM::Point2D(ball.getX(),ball.getY()),SIM::Point2D(target.getX(), target.getY()),0.3);
                    worldModel->getRVSender()->drawCircle("AdjsutementBehindMeBad",adjustedPos.x, adjustedPos.y,0.1,0,0,0); 
                    worldModel->getRVSender()->drawLine("GOAL Traj",mydesiredposition.getX(), mydesiredposition.getY(),adjustedPos.x, adjustedPos.y,0,0,0); 
                    return SmartGoToTargetWithPrecision(VecPosition(adjustedPos.x,adjustedPos.y,0),0,0.3); 
                }
                else{
                    return kickBallFat(shoot_kick, target, mydesiredposition); // Basic kick
                }

 
            }
            else{
                //Future Work can be to extend this in a manner such that we can move the ball up field
                //The Future Is Now

                if(!(hasKickLatched)){

                    hasKickLatched = true;
                    kickLatchCount = 0;


                    VecPosition upFieldPos = DetermineGoodKickToLocation(_playerNumber, _myPos);
                    worldModel->getRVSender()->drawCircle("upFieldPos",upFieldPos.getX(), upFieldPos.getY(),0.1,0,1,1); //GB
                    worldModel->getRVSender()->drawLine("upFieldPosLine",_myPos.getX(), _myPos.getY(),upFieldPos.getX(), upFieldPos.getY(),0,1,1); //GB
                        


                    target = DetermineValidPassPositionPlayerCentricFAT(TeamDistToOppGoal,TeamDistToBall, target, FAT_SAFE_PASS_DISTANCE, _playerNumber, true);

                    if(target != NULL){

                        worldModel->getRVSender()->drawCircle("Kicker",_myPos.getX(), _myPos.getY(),0.3,0,0,1); //BLUE
                        worldModel->getRVSender()->drawLine("KICKTARGET",_myPos.getX(), _myPos.getY(),target.getX(), target.getY(),RVSender::YELLOW); //YELLOW
                        //worldModel->getRVSender()->drawText("KICKTARGET",std::to_string(target.getX())+","+std::to_string(target.getY()),_myPos.getX(),_myPos.getY()+1,0,0,0);
                        worldModel->getRVSender()->drawPoint("ball", target.getX(), target.getY(), 10.0f, RVSender::ORANGE);    

                        if(isBetweenTargetAndBall(target,_myPos,0.3)){
                            worldModel->getRVSender()->drawCircle("BehindMeBad",_myPos.getX(), _myPos.getY(),0.4,1,0,1); //PINK
                            SIM::Point2D origin = SIM::Point2D(_myPos.getX(),_myPos.getY());
                            SIM::Point2D adjustedPos = origin.findShortestPositionNearBall(origin,SIM::Point2D(ball.getX(),ball.getY()),SIM::Point2D(target.getX(), target.getY()),0.3);
                            worldModel->getRVSender()->drawCircle("AdjsutementBehindMeBad",adjustedPos.x, adjustedPos.y,0.1,0,0,0); 
                            worldModel->getRVSender()->drawLine("GOAL Traj",mydesiredposition.getX(), mydesiredposition.getY(),adjustedPos.x, adjustedPos.y,0,0,0); 
                            return SmartGoToTargetWithPrecision(VecPosition(adjustedPos.x,adjustedPos.y,0),0,0.3); 
                    
                        }
                        return kickBallFat(ground_pass, target, mydesiredposition); // Basic kick

                    }
                    else{
                        worldModel->getRVSender()->drawCircle("Stand",_myPos.getX(), _myPos.getY(),0.3,1,0,0); //RED
                        return kickBallFat(ground_pass, upFieldPos, mydesiredposition); // Basic kick
                    }
                }
                else{
                    if(kickLatchCount < MAX_KICK_LATCH_COUNT){
                        kickLatchCount++;
                        return SKILL_STAND;
                    }
                    else{
                        kickLatchCount = 0;
                        hasKickLatched = false;
                    }
                }
            }

        }
        else{

            //CHECK IF I AM A RECIEVER
            int passid = DetermineValidPassIDPlayerCentric(TeamDistToOppGoal,TeamDistToBall, target, FAT_SAFE_PASS_DISTANCE);

            if(passid == _playerNumber){
                 worldModel->getRVSender()->drawCircle("RECIVER",_myPos.getX(), _myPos.getY(),0.2,1,1,1); //WHITE
                 return SKILL_STAND;
            }
            else{
               worldModel->getRVSender()->drawCircle("WALKER",_myPos.getX(), _myPos.getY(),0.3,0,1,0); //GREEN
               worldModel->getRVSender()->drawCircle("DESIRED",mydesiredposition.getX(), mydesiredposition.getY(),0.1,0,1,0); //GREEN
               return SmartGoToTarget(mydesiredposition,1); 
           }


            // if(isPassReciever(TeamDistToBall)){
            //     worldModel->getRVSender()->drawCircle("Stand",_myPos.getX(), _myPos.getY(),0.3,1,0,0); //RED
            //     return SKILL_STAND;

            // }
            


            worldModel->getRVSender()->drawCircle("WALKER",_myPos.getX(), _myPos.getY(),0.3,0,1,0); //GREEN
            return SmartGoToTarget(mydesiredposition,1); 
        }
    // }
    // else{
    //     if(isClosestTeam( _playerNumber, TeamDistToBall)){
    //         worldModel->getRVSender()->drawCircle("Close DEF",_myPos.getX(), _myPos.getY(),0.3,0,0,0); //Black
    //         return SmartGoToTarget(ball,0.3); 
    //     }
    //     else{
    //         worldModel->getRVSender()->drawCircle("WALKER",_myPos.getX(), _myPos.getY(),0.3,0,1,0); //GREEN

    //         return SmartGoToTarget(mydesiredposition,1); 
    //     }


    // }




}

SkillType NaoBehavior::WitsDecisionTreeBehaviour2023(){
    worldModel->getRVSender()->clear();

    int _playerNumber = worldModel->getUNum();

    std::string SpNum = std::to_string(_playerNumber);
    ofstream fw("./BehaviourLogs/"+SpNum+".txt", std::ofstream::out);

   
    int _playMode = worldModel->getPlayMode();
    WorldObject* _gameObject = worldModel->getWorldObject(_playerNumber);
    VecPosition _myPos = worldModel->getMyPosition();
    _myPos.setZ(0);
    int _side = worldModel->getSide();
    vector<pair<double,int > > TeamDistToBall = GenerateTeamToTargetDistanceVector(_playerNumber, ball);
    vector<pair<double,int > > OppDistToBall = GenerateOppToTargetDistanceVector(ball);
    vector<pair<double,int > > TeamDistToOppGoal = GenerateTeamToTargetDistanceVector(_playerNumber, VecPosition(HALF_FIELD_X, 0, 0));

    int regionX = getXFieldRegion(ball);
    int regionY = getYFieldRegion(ball);

    // if(_playerNumber == 1){
    //     return kickBall(KICK_IK, VecPosition(1, 3, 0));
    // }
    // else{
    //     return SKILL_STAND;
    // }


    
    



    if(_playerNumber == 1){

        if(visualise){
            worldModel->getRVSender()->drawLine("line1", -5,-HALF_FIELD_Y, -5,HALF_FIELD_Y, 0,0,0);
            worldModel->getRVSender()->drawLine("line2", 5,-HALF_FIELD_Y, 5,HALF_FIELD_Y, 0,0,0);

            worldModel->getRVSender()->drawLine("line3", -HALF_FIELD_X,5, HALF_FIELD_X,5, 0,0,0);
            worldModel->getRVSender()->drawLine("line4", -HALF_FIELD_X,-5, HALF_FIELD_X,-5, 0,0,0);

            worldModel->getRVSender()->drawText("Bottom-5",std::to_string(-5)+","+std::to_string(int(-HALF_FIELD_Y)),-5,-HALF_FIELD_Y,0,0,0);
            worldModel->getRVSender()->drawText("Top-5",std::to_string(-5)+","+std::to_string(int(HALF_FIELD_Y)),-5,HALF_FIELD_Y,0,0,0);
            worldModel->getRVSender()->drawText("Bottom5",std::to_string(5)+","+std::to_string(int(-HALF_FIELD_Y)),5,-HALF_FIELD_Y,0,0,0);
            worldModel->getRVSender()->drawText("Top5",std::to_string(5)+","+std::to_string(int(HALF_FIELD_Y)),5,HALF_FIELD_Y,0,0,0);

            worldModel->getRVSender()->drawText("Centre",std::to_string(int(FIELD_CENTER_X))+","+std::to_string(int(FIELD_CENTER_Y)),FIELD_CENTER_X,FIELD_CENTER_Y,0,0,0);
            worldModel->getRVSender()->drawText("Right",std::to_string(int(HALF_FIELD_X))+","+std::to_string(int(FIELD_CENTER_Y)),HALF_FIELD_X,FIELD_CENTER_Y,0,0,0);
            worldModel->getRVSender()->drawText("Top",std::to_string(int(FIELD_CENTER_X))+","+std::to_string(int(HALF_FIELD_Y)),FIELD_CENTER_X,HALF_FIELD_Y,0,0,0);
            worldModel->getRVSender()->drawText("Left",std::to_string(int(-HALF_FIELD_X))+","+std::to_string(int(FIELD_CENTER_Y)),-HALF_FIELD_X,FIELD_CENTER_Y,0,0,0);
            worldModel->getRVSender()->drawText("Bottom",std::to_string(int(FIELD_CENTER_X))+","+std::to_string(int(-HALF_FIELD_Y)),FIELD_CENTER_X,-HALF_FIELD_Y,0,0,0);
        

            
        }


        // for (int i = 0; i < TeamDistToBall.size(); i++) {
        //     cout << "("
        //         << TeamDistToBall[i].first << ", "
        //         << TeamDistToBall[i].second << ") ";
        // }
        // cout << "\n";
        
        // for (int i = 0; i < OppDistToBall.size(); i++) {
        //         cout << "("
        //             << OppDistToBall[i].first << ", "
        //             << OppDistToBall[i].second << ") ";
        // }
        // cout << "\n";
    
    
    }

    for (int i = 0; i < TeamDistToBall.size(); i++) {
        string line = std::to_string(TeamDistToBall[i].first) + " : " + std::to_string(TeamDistToBall[i].second);
        string linename = "Distance" + std::to_string(i);
        double offset = i*0.5;

        if(visualise && _playerNumber==1){
            //worldModel->getRVSender()->drawText(linename,line,-16,HALF_FIELD_Y-offset,1,0,0);
        }

        if(log){
            if (fw.is_open())
            {
                fw << line << "\n";
            }
        }


    }

    DetermineUsePass(OppDistToBall);    
    DetermineUseGoalieCatchBall(_playerNumber);


    vector<VecPosition> ImportantPositions;
    ImportantPositions = GenerateImportantPositions(_playMode, _side, _playerNumber, regionX, regionY);

    vector<vector<pair<double,int > > > PreferenceToPointArray = GeneratePreferenceArrayForTeam(_playerNumber,ImportantPositions);

    vector<int> PointPreferences = stableMarriage(PreferenceToPointArray);

    /* for(int i=0;i<NUM_AGENTS;i++){
        cout << i << "," << PointPreferences[i] << "|";
    }
    cout << "\n"; */

    int mydesiredpositionIndex = PointPreferences[_playerNumber-1];
    //int mydesiredpositionIndex = PreferenceToPointArray[_playerNumber][0].second;  // only go to closest important position no optomisation

    VecPosition mydesiredposition = ImportantPositions[mydesiredpositionIndex];

    if(visualise){
        worldModel->getRVSender()->drawLine("Line_to_Desired_Target:"+SpNum, _myPos.getX(),_myPos.getY(),mydesiredposition.getX(), mydesiredposition.getY(),1,0,0); //RED
        worldModel->getRVSender()->drawCircle(SpNum,mydesiredposition.getX(), mydesiredposition.getY(),0.5,1,0,0); //RED
    }
    //VecPosition targetDebugger = GetDesiredTargetAdvanced(_playerNumber,_myPos,regionX,regionY,TeamDistToBall);
    //worldModel->getRVSender()->drawCircle(SpNum+"KickTarget",targetDebugger.getX(), targetDebugger.getY(),0.2,0,1,0); //RED
    //return SmartGoToTarget(mydesiredposition,0.5);

    if(_playMode == PM_KICK_OFF_LEFT){
        if(worldModel->getSide() == SIDE_LEFT){
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                
                return doRLKick(VecPosition(-5, 0, 0)); // mike 2023/07/02
                return kickBall(KICK_FORWARD, VecPosition(-2.5, 4, 0));
            }
        }
        else{
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
    }
    else if(_playMode == PM_KICK_OFF_RIGHT){
        if(worldModel->getSide() == SIDE_RIGHT){
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                return doRLKick(VecPosition(-5, 0, 0)); // mike 2023/07/02
                return kickBall(KICK_FORWARD, VecPosition(-2.5, 4, 0));
            }
        }
        else{
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
    }
    else if(_playMode == PM_FREE_KICK_LEFT || (_playMode == PM_KICK_IN_LEFT) || (_playMode == PM_GOAL_KICK_LEFT)) {
                   // cout << worldModel->getSide() << endl;

        if(worldModel->getSide() == SIDE_LEFT){
                               // cout << "side == right" << endl;
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                    //cout << _playerNumber << "is closest" << endl;
                VecPosition target = getClosestTeammatePos(_playerNumber,TeamDistToBall);
                target = OptimalPassPosition(_playerNumber, VecPosition(0,0,0),PointPreferences,ImportantPositions);
                return doRLKick(target); // mike 2023/07/02
                return kickBall(KICK_IK, target);
            }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            }
        }
        else{
             cout << "side == left" << endl;
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
    }
    else if(_playMode == PM_FREE_KICK_RIGHT || (_playMode == PM_KICK_IN_RIGHT) || (_playMode == PM_GOAL_KICK_RIGHT)){
        if(worldModel->getSide() == SIDE_RIGHT){
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                    //cout << _playerNumber << "is closest" << endl;
                VecPosition target = getClosestTeammatePos(_playerNumber,TeamDistToBall);
                target = OptimalPassPosition(_playerNumber, VecPosition(0,0,0),PointPreferences,ImportantPositions);
                return doRLKick(target); // mike 2023/07/02
                return kickBall(KICK_IK, target);
            }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            }
        }
        else{
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
        
    }
    else if((_playMode == PM_GAME_OVER) || (_playMode == PM_GOAL_LEFT) || (_playMode == PM_GOAL_RIGHT)){
            return SKILL_STAND;
    }  
    



    if(isOffensiveStrat(_playerNumber,TeamDistToBall,OppDistToBall)){

        if(isClosestTeam( _playerNumber, TeamDistToBall)){

            if(isShootingRange(_myPos, SHOOTING_RANGE)){
                //usePass = true;
                VecPosition Adjustedtarget = DetermineShootAtGoalPosition(_myPos, _playerNumber, TeamDistToBall); 
                //return doRLKick(Adjustedtarget); // mike 2023/07/02

                return DetermineAppropriateRLKick(_myPos, OppDistToBall, Adjustedtarget);  
            }
            else{
                if (hasKickLatched){
                    kickLatchCount++;
                    if (kickLatchCount >= MAX_KICK_LATCH_COUNT){
                        hasKickLatched = false;
                        kickLatchCount = 0;
                    }
                    return DetermineAppropriateRLKick(_myPos, OppDistToBall, kickLatchTarget); 
                }
                else{
                    if (_myPos.getDistanceTo(ball) < 1.5){
                        hasKickLatched = true;
                        kickLatchCount = 0;
                    }
                    VecPosition target = DetermineValidPassPositionPlayerCentric(_myPos, TeamDistToOppGoal,TeamDistToBall, target, SAFE_PASS_DISTANCE, _playerNumber, false);

                    if(target != NULL){
                        if (hasKickLatched) kickLatchTarget = target;
                        return DetermineAppropriateRLKick(_myPos, OppDistToBall, target); 
                    }
                    else{
                        
                        VecPosition target = GetDesiredTargetAdvanced(_playerNumber,_myPos,regionX,regionY,TeamDistToBall);
                        target = OptimalPassPosition(_playerNumber, target, PointPreferences, ImportantPositions);

                        
                        VecPosition Adjustedtarget = DetermineValidPassRecipient(target,_myPos,_playerNumber,TeamDistToBall,false);
                        if (hasKickLatched) kickLatchTarget = Adjustedtarget;
                        return DetermineAppropriateRLKick(_myPos, OppDistToBall, Adjustedtarget); 
                    }
                }
            }
        }
        else{

            float distance = _myPos.getDistanceTo(mydesiredposition);

            if(distance < 0.5){

                return FaceBall(_myPos);
                }
            else{

                return SmartGoToTarget(mydesiredposition,1);
            } 
        }

    }else{

        if(isClosestTeam( _playerNumber, TeamDistToBall)){
           
            if(ball.getX() >_myPos.getX()){
                double minOppDis = OppDistToBall[0].first; 
                if(minOppDis > 2){
                    
                    VecPosition target = GetDesiredTargetAdvanced(_playerNumber,_myPos,regionX,regionY,TeamDistToBall);
                    target = OptimalPassPosition(_playerNumber, target, PointPreferences, ImportantPositions);

                    return DetermineAppropriateRLKick(_myPos, OppDistToBall, target); 
                }
            }

            VecPosition midgoalball = ball.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            VecPosition midmidball = ball.Midpoint(midgoalball);
            VecPosition midmidmidball = ball.Midpoint(midmidball);
            midmidmidball = ValidateFieldPos(midmidmidball);

            if(isBetweenTargetAndBall(midmidball,_myPos,0.3)){
                VecPosition target = GetDesiredTargetAdvanced(_playerNumber,_myPos,regionX,regionY,TeamDistToBall);
                target = OptimalPassPosition(_playerNumber, target, PointPreferences, ImportantPositions);

                return DetermineAppropriateRLKick(_myPos, OppDistToBall, target);
            }

            if(atDestination(_myPos, midmidmidball)){
                return DetermineAppropriateRLKick(_myPos, OppDistToBall, VecPosition(HALF_FIELD_X, 0, 0));

            }
            else{

                return SmartGoToTarget(midmidmidball,0.3); 
            }
        }
        else if (AmITheNthClosestTeammateToBall(TeamDistToBall, _playerNumber,2)){

            VecPosition midgoalball = ball.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            VecPosition midmidball = ball.Midpoint(midgoalball);
            VecPosition midmidmidball = ball.Midpoint(midmidball);
            midmidmidball = ValidateFieldPos(midmidmidball);

            float distance = _myPos.getDistanceTo(midmidmidball);
            if(distance < 0.5){
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(midmidmidball,1);
            }
        }
        else{
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                return FaceBall(_myPos);
            }
            else{
                return SmartGoToTargetWithPrecisionAndFaceSpecificLocation(mydesiredposition,1,1, _myPos, ball);
            } 
        }
    }
    return SKILL_STAND;
}

SkillType NaoBehavior::WitsStandWithGoalieCatch(){

    worldModel->getRVSender()->clear();

    int _playerNumber = worldModel->getUNum();
    vector<pair<double,int > > TeamDistToBall = GenerateTeamToTargetDistanceVector(_playerNumber, ball);
    vector<pair<double,int > > OppDistToBall = GenerateOppToTargetDistanceVector(ball);
    DetermineUsePass(OppDistToBall);    
    DetermineUseGoalieCatchBall(_playerNumber);

    if(_playerNumber == 1){
        return SKILL_LATERAL_DIVE_LEFT;
    }
    else{
            return SKILL_STAND;
    }


}

SkillType NaoBehavior::WitsDecisionTreeBehaviour() {

    // if(PM_GAME_OVER){
    //     return SKILL_STAND;
    // }


    worldModel->getRVSender()->clear();

    int _playerNumber = worldModel->getUNum();

    std::string SpNum = std::to_string(_playerNumber);
    ofstream fw("./BehaviourLogs/"+SpNum+".txt", std::ofstream::out);

   
    int _playMode = worldModel->getPlayMode();
    WorldObject* _gameObject = worldModel->getWorldObject(_playerNumber);
    VecPosition _myPos = worldModel->getMyPosition();
    _myPos.setZ(0);
    int _side = worldModel->getSide();
    vector<pair<double,int > > TeamDistToBall = GenerateTeamToTargetDistanceVector(_playerNumber, ball);
    vector<pair<double,int > > OppDistToBall = GenerateOppToTargetDistanceVector(ball);
    int regionX = getXFieldRegion(ball);
    int regionY = getYFieldRegion(ball);

    // if(_playerNumber == 1){
    //     return kickBall(KICK_IK, VecPosition(1, 3, 0));
    // }
    // else{
    //     return SKILL_STAND;
    // }


    
    



    if(_playerNumber == 1){

        if(visualise){
            worldModel->getRVSender()->drawLine("line1", -5,-HALF_FIELD_Y, -5,HALF_FIELD_Y, 0,0,0);
            worldModel->getRVSender()->drawLine("line2", 5,-HALF_FIELD_Y, 5,HALF_FIELD_Y, 0,0,0);

            worldModel->getRVSender()->drawLine("line3", -HALF_FIELD_X,5, HALF_FIELD_X,5, 0,0,0);
            worldModel->getRVSender()->drawLine("line4", -HALF_FIELD_X,-5, HALF_FIELD_X,-5, 0,0,0);

            worldModel->getRVSender()->drawText("Bottom-5",std::to_string(-5)+","+std::to_string(int(-HALF_FIELD_Y)),-5,-HALF_FIELD_Y,0,0,0);
            worldModel->getRVSender()->drawText("Top-5",std::to_string(-5)+","+std::to_string(int(HALF_FIELD_Y)),-5,HALF_FIELD_Y,0,0,0);
            worldModel->getRVSender()->drawText("Bottom5",std::to_string(5)+","+std::to_string(int(-HALF_FIELD_Y)),5,-HALF_FIELD_Y,0,0,0);
            worldModel->getRVSender()->drawText("Top5",std::to_string(5)+","+std::to_string(int(HALF_FIELD_Y)),5,HALF_FIELD_Y,0,0,0);

            worldModel->getRVSender()->drawText("Centre",std::to_string(int(FIELD_CENTER_X))+","+std::to_string(int(FIELD_CENTER_Y)),FIELD_CENTER_X,FIELD_CENTER_Y,0,0,0);
            worldModel->getRVSender()->drawText("Right",std::to_string(int(HALF_FIELD_X))+","+std::to_string(int(FIELD_CENTER_Y)),HALF_FIELD_X,FIELD_CENTER_Y,0,0,0);
            worldModel->getRVSender()->drawText("Top",std::to_string(int(FIELD_CENTER_X))+","+std::to_string(int(HALF_FIELD_Y)),FIELD_CENTER_X,HALF_FIELD_Y,0,0,0);
            worldModel->getRVSender()->drawText("Left",std::to_string(int(-HALF_FIELD_X))+","+std::to_string(int(FIELD_CENTER_Y)),-HALF_FIELD_X,FIELD_CENTER_Y,0,0,0);
            worldModel->getRVSender()->drawText("Bottom",std::to_string(int(FIELD_CENTER_X))+","+std::to_string(int(-HALF_FIELD_Y)),FIELD_CENTER_X,-HALF_FIELD_Y,0,0,0);
        

            
        }


        // for (int i = 0; i < TeamDistToBall.size(); i++) {
        //     cout << "("
        //         << TeamDistToBall[i].first << ", "
        //         << TeamDistToBall[i].second << ") ";
        // }
        // cout << "\n";
        
        // for (int i = 0; i < OppDistToBall.size(); i++) {
        //         cout << "("
        //             << OppDistToBall[i].first << ", "
        //             << OppDistToBall[i].second << ") ";
        // }
        // cout << "\n";
    
    
    }

    for (int i = 0; i < TeamDistToBall.size(); i++) {
        string line = std::to_string(TeamDistToBall[i].first) + " : " + std::to_string(TeamDistToBall[i].second);
        string linename = "Distance" + std::to_string(i);
        double offset = i*0.5;

        if(visualise && _playerNumber==1){
            //worldModel->getRVSender()->drawText(linename,line,-16,HALF_FIELD_Y-offset,1,0,0);
        }

        if(log){
            if (fw.is_open())
            {
                fw << line << "\n";
            }
        }


    }

    DetermineUsePass(OppDistToBall);    
    DetermineUseGoalieCatchBall(_playerNumber);


    vector<VecPosition> ImportantPositions;
    ImportantPositions = GenerateImportantPositions(_playMode, _side, _playerNumber, regionX, regionY);

    vector<vector<pair<double,int > > > PreferenceToPointArray = GeneratePreferenceArrayForTeam(_playerNumber,ImportantPositions);

    vector<int> PointPreferences = stableMarriage(PreferenceToPointArray);

    /* for(int i=0;i<NUM_AGENTS;i++){
        cout << i << "," << PointPreferences[i] << "|";
    }
    cout << "\n"; */

    int mydesiredpositionIndex = PointPreferences[_playerNumber-1];
    //int mydesiredpositionIndex = PreferenceToPointArray[_playerNumber][0].second;  // only go to closest important position no optomisation

    VecPosition mydesiredposition = ImportantPositions[mydesiredpositionIndex];

    if(visualise){
        worldModel->getRVSender()->drawLine("Line_to_Desired_Target:"+SpNum, _myPos.getX(),_myPos.getY(),mydesiredposition.getX(), mydesiredposition.getY(),1,0,0); //RED
        worldModel->getRVSender()->drawCircle(SpNum,mydesiredposition.getX(), mydesiredposition.getY(),0.5,1,0,0); //RED
    }
    //VecPosition targetDebugger = GetDesiredTargetAdvanced(_playerNumber,_myPos,regionX,regionY,TeamDistToBall);
    //worldModel->getRVSender()->drawCircle(SpNum+"KickTarget",targetDebugger.getX(), targetDebugger.getY(),0.2,0,1,0); //RED
    //return SmartGoToTarget(mydesiredposition,0.5);

    if(_playMode == PM_KICK_OFF_LEFT){
        if(worldModel->getSide() == SIDE_LEFT){
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                
                return doRLKick(VecPosition(-5, 0, 0), "normal"); // mike 2023/07/02
                return kickBall(KICK_FORWARD, VecPosition(-2.5, 4, 0));
            }
        }
        else{
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
    }
    else if(_playMode == PM_KICK_OFF_RIGHT){
        if(worldModel->getSide() == SIDE_RIGHT){
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                return doRLKick(VecPosition(-5, 0, 0), "normal"); // mike 2023/07/02
                return kickBall(KICK_FORWARD, VecPosition(-2.5, 4, 0));
            }
        }
        else{
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
    }
    else if(_playMode == PM_FREE_KICK_LEFT || (_playMode == PM_KICK_IN_LEFT) || (_playMode == PM_GOAL_KICK_LEFT)) {
                   // cout << worldModel->getSide() << endl;

        if(worldModel->getSide() == SIDE_LEFT){
                               // cout << "side == right" << endl;
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                    //cout << _playerNumber << "is closest" << endl;
                VecPosition target = getClosestTeammatePos(_playerNumber,TeamDistToBall);
                target = OptimalPassPosition(_playerNumber, VecPosition(0,0,0),PointPreferences,ImportantPositions);
                return doRLKick(target, "long"); // mike 2023/07/02
                return kickBall(KICK_IK, target);
            }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            }
        }
        else{
             cout << "side == left" << endl;
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
    }
    else if(_playMode == PM_FREE_KICK_RIGHT || (_playMode == PM_KICK_IN_RIGHT) || (_playMode == PM_GOAL_KICK_RIGHT)){
        if(worldModel->getSide() == SIDE_RIGHT){
            if(isClosestTeam( _playerNumber, TeamDistToBall)){
                    //cout << _playerNumber << "is closest" << endl;
                VecPosition target = getClosestTeammatePos(_playerNumber,TeamDistToBall);
                target = OptimalPassPosition(_playerNumber, VecPosition(0,0,0),PointPreferences,ImportantPositions);
                return doRLKick(target, "long");
                return kickBall(KICK_IK, target);
            }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            }
        }
        else{
            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    return FaceBall(_myPos);
                }
            else{
                return SmartGoToTarget(mydesiredposition,0.5);
            } 
        }
        
    }
    else if((_playMode == PM_GAME_OVER) || (_playMode == PM_GOAL_LEFT) || (_playMode == PM_GOAL_RIGHT)){
            return SKILL_STAND;
    }

            
    
    if(isOffensiveStrat(_playerNumber,TeamDistToBall,OppDistToBall)){

        if(log){
            if (fw.is_open())
            {
                fw << "OFFENSIVESTRAT" << "\n";
            }
        }


        if(isClosestTeam( _playerNumber, TeamDistToBall)){

            if(log){
                if (fw.is_open())
                {
                    fw << "OFFENSIVESTRAT/CLOSESTTEAM" << "\n";
                }
            }


            VecPosition target = GetDesiredTargetAdvanced(_playerNumber,_myPos,regionX,regionY,TeamDistToBall);
            target = OptimalPassPosition(_playerNumber, target, PointPreferences, ImportantPositions);

            if(log){
                if (fw.is_open())
                {
                    fw << "Initial Target : " + target.toString() << "\n";
                }
            }


            if(visualise){
                worldModel->getRVSender()->drawCircle("Closest TeamMate:"+SpNum,_myPos.getX(), _myPos.getY(),0.6,0,0,1); //BLUE
                //worldModel->getRVSender()->drawLine(_myPos.getX(),_myPos.getY(),ball.getX(), ball.getY(),0,0,1); //BLUE
                //worldModel->getRVSender()->drawText("OFFENSIVE Region (X,Y)","OFFENSIVE Region (X,Y) = " +std::to_string(regionX)+","+std::to_string(regionY),0,-11,0,0,0);
                worldModel->getRVSender()->drawCircle("targetclose", target.getX(), target.getY(), 1, 0,1,0.5); //green-blue
                worldModel->getRVSender()->drawLine("TargetLine",target.getX(),target.getY(),ball.getX(), ball.getY(),0,1,0.5); //green-blue
            }

            if(isShootingRange(_myPos, SHOOTING_RANGE)){

                if(log){
                    if (fw.is_open())
                    {
                        fw << "OFFENSIVESTRAT/CLOSESTTEAM/SHOOTINGRANGE" << "\n";
                    }
                }

                //usePass = true;

                VecPosition Adjustedtarget = DetermineShootAtGoalPosition(_myPos, _playerNumber, TeamDistToBall); 

                if(visualise){
                    worldModel->getRVSender()->drawCircle("shot target", Adjustedtarget.getX(), Adjustedtarget.getY(), 1, 0,1,0.5); //green-blue
                }
                if(log){
                    if (fw.is_open())
                    {
                        fw << "Adjusted Target for Shooting : " + Adjustedtarget.toString() << "\n";
                        fw << "RETURN DETERMINE APPROPRIATE KICK" << "\n";
                    }
                }
                //return doRLKick(Adjustedtarget); // mike 2023/07/02
                return DetermineAppropriateRLKick(_myPos, OppDistToBall, Adjustedtarget);  
                //return DetermineAppropriateKick(_myPos, OppDistToBall, Adjustedtarget);   

                /* if(abs(ball.getX() - HALF_FIELD_X) < 1){

                    return DetermineAppropriateKick(_myPos, OppDistToBall, VecPosition(HALF_FIELD_X+0.3, 0, 0));
                    //return kickBall(KICK_FORWARD, VecPosition(HALF_FIELD_X, 0, 0)); // Basic kick
                }
                else if (BallinOppGoalMouth()){
                    return DetermineAppropriateKick(_myPos, OppDistToBall, VecPosition(HALF_FIELD_X+5, ball.getY(), 0));
                }
                return DetermineAppropriateKick(_myPos, OppDistToBall, target);
                //return kickBall(KICK_FORWARD, target); */
            }
            else{

                if(log){
                    if (fw.is_open())
                    {
                        fw << "OFFENSIVESTRAT/CLOSESTTEAM/notSHOOTINGRANGE" << "\n";
                    }
                }

                
                VecPosition Adjustedtarget = DetermineValidPassRecipient(target,_myPos,_playerNumber,TeamDistToBall,false);

                if(log){
                    if (fw.is_open())
                    {
                        fw << "Adjusted Target for Passing : " + Adjustedtarget.toString() << "\n";
                    }
                }

                
                if(visualise){
                    worldModel->getRVSender()->drawCircle("Adjustedtargetclose", Adjustedtarget.getX(), Adjustedtarget.getY(), 1, 0,0.5,1); //blue
                    worldModel->getRVSender()->drawLine("AdjustedTargetLine",Adjustedtarget.getX(),Adjustedtarget.getY(),ball.getX(), ball.getY(),0,0.5,1); //blue
                }

                if(isAnyTeammateInNeighbourhood(_playerNumber, Adjustedtarget, 3)){

                    if(log){
                        if (fw.is_open())
                        {
                            fw << "OFFENSIVESTRAT/CLOSESTTEAM/notSHOOTINGRANGE/TEAMMATEINNEIGHBOURHOOD" << "\n";
                            fw << "RETURN DETERMINE APPROPRIATE KICK" << "\n";
                        }
                    }

                    if(visualise){
                        worldModel->getRVSender()->drawCircle(SpNum,_myPos.getX(),_myPos.getY(),0.2,1,0,1); //PINK
                    }
                    if (visualise){
                        worldModel->getRVSender()->drawLine("p", _myPos.getX(),_myPos.getY(), Adjustedtarget.getX(), Adjustedtarget.getY(), RVSender::RED);

                        worldModel->getRVSender()->drawCircle("p", Adjustedtarget.getX(), Adjustedtarget.getY(),0.2, RVSender::RED);
                    }
                    // return doRLKick(Adjustedtarget); // mike 2023/07/02
                    return DetermineAppropriateRLKick(_myPos, OppDistToBall, Adjustedtarget); 
                    //return DetermineAppropriateKick(_myPos, OppDistToBall, Adjustedtarget);
                    //return kickBall(KICK_FORWARD, target);
                }

                if(log){
                    if (fw.is_open())
                    {
                        fw << "OFFENSIVESTRAT/CLOSESTTEAM/notSHOOTINGRANGE/notTEAMMATEINNEIGHBOURHOOD" << "\n";
                        fw << "RETURN DETERMINE APPROPRIATE KICK" << "\n";
                    }
                }
                Adjustedtarget = DetermineGoodKickToLocation(_playerNumber, _myPos, 4);

                if (visualise){
                    worldModel->getRVSender()->drawLine("p", _myPos.getX(),_myPos.getY(), Adjustedtarget.getX(), Adjustedtarget.getY(), RVSender::BLUE);
                    worldModel->getRVSender()->drawCircle("p", Adjustedtarget.getX(), Adjustedtarget.getY(),0.2, RVSender::BLUE);
                }
                return DetermineAppropriateRLKick(_myPos, OppDistToBall, Adjustedtarget); 
                //return kickBall(KICK_DRIBBLE, Adjustedtarget);
                //return DetermineAppropriateKick(_myPos, OppDistToBall, Adjustedtarget);
                //return kickBall(KICK_DRIBBLE, target);
            }
        }
        else{

            if(log){
                if (fw.is_open())
                {
                    fw << "OFFENSIVESTRAT/notCLOSESTTEAM/" << "\n";
                }
            }


            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(visualise){
                worldModel->getRVSender()->drawLine("DesiredPosFormation",_myPos.getX(),_myPos.getY(),mydesiredposition.getX(), mydesiredposition.getY(),1,0,0); //RED
            }
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }
                    if(log){
                        if (fw.is_open())
                        {
                            fw << "RETURN FACE BALL" << "\n";
                        }
                    }


                    return FaceBall(_myPos);
                }
            else{

                if(log){
                    if (fw.is_open())
                    {
                        fw << "RETURN GO TO TARGET: " + mydesiredposition.toString() << "\n";
                    }
                }

                return SmartGoToTarget(mydesiredposition,1);
            } 
        }

    }else{

        if(log){
            if (fw.is_open())
            {
                fw << "notOFFENSIVESTRAT" << "\n";
            }
        }

        if(isClosestTeam( _playerNumber, TeamDistToBall)){

            if(log){
                if (fw.is_open())
                {
                    fw << "notOFFENSIVESTRAT/CLOSESTTEAM/" << "\n";
                }
            }

            if(visualise){
                worldModel->getRVSender()->drawCircle(SpNum,_myPos.getX(), _myPos.getY(),0.6,0,0,1); //BLUE
                //worldModel->getRVSender()->drawText("DEFENSIVE Region (X,Y)","DEFENSIVE Region (X,Y) = " +std::to_string(regionX)+","+std::to_string(regionY),0,-11,0,0,0);
            }
           
            if(ball.getX() >_myPos.getX()){
                double minOppDis = OppDistToBall[0].first; 
                if(minOppDis > 2){
                    
                    VecPosition target = GetDesiredTargetAdvanced(_playerNumber,_myPos,regionX,regionY,TeamDistToBall);
                    target = OptimalPassPosition(_playerNumber, target, PointPreferences, ImportantPositions);
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("AgroDef",_myPos.getX(), _myPos.getY(),0.2,0,0,0); //ORANGE
                        worldModel->getRVSender()->drawLine("AgroDefline",_myPos.getX(),_myPos.getY(),target.getX(), target.getY(),0,0,0); //blue
                    }

                    if(log){
                        if (fw.is_open())
                        {
                            fw << "RETURN KICK BALL DRIBBLE: " + target.toString() << "\n";
                        }
                    }

                    return DetermineAppropriateRLKick(_myPos, OppDistToBall, target); 
                    //return kickBall(KICK_DRIBBLE, target);
                }
            }


            VecPosition midgoalball = ball.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            VecPosition midmidball = ball.Midpoint(midgoalball);
            VecPosition midmidmidball = ball.Midpoint(midmidball);
            midmidmidball = ValidateFieldPos(midmidmidball);
            if(visualise){
                worldModel->getRVSender()->drawCircle("Target2",midmidmidball.getX(), midmidmidball.getY(),0.3,0.5,0,1); //VIOLET
            }

            if(isBetweenTargetAndBall(midmidball,_myPos,0.3)){
                VecPosition target = GetDesiredTargetAdvanced(_playerNumber,_myPos,regionX,regionY,TeamDistToBall);
                target = OptimalPassPosition(_playerNumber, target, PointPreferences, ImportantPositions);
                if(visualise){
                    worldModel->getRVSender()->drawCircle("Tisbetween1",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    worldModel->getRVSender()->drawLine("Tisbetween1line",_myPos.getX(),_myPos.getY(),target.getX(), target.getY(),0,0,1); //blue
                }

                if(log){
                    if (fw.is_open())
                    {
                        fw << "RETURN KICK BALL DRIBBLE: " + target.toString() << "\n";
                    }
                }
                return DetermineAppropriateRLKick(_myPos, OppDistToBall, target);
                //return kickBall(KICK_DRIBBLE, target);
            }


            if(atDestination(_myPos, midmidmidball)){
                // return doRLKick(VecPosition(HALF_FIELD_X, 0, 0)); // mike 2023/07/02
                return DetermineAppropriateRLKick(_myPos, OppDistToBall, VecPosition(HALF_FIELD_X, 0, 0));
                //return DetermineAppropriateKick(_myPos, OppDistToBall, VecPosition(HALF_FIELD_X, 0, 0));
                //return kickBall(KICK_DRIBBLE, VecPosition(HALF_FIELD_X, 0, 0));
            }
            else{

                if(log){
                    if (fw.is_open())
                    {
                        fw << "RETURN GO TO TARGET: " + midmidmidball.toString() << "\n";
                    }
                }

                return SmartGoToTarget(midmidmidball,0.3); 
            }
        
        }
        else if (AmITheNthClosestTeammateToBall(TeamDistToBall, _playerNumber,2)){

            if(log){
                if (fw.is_open())
                {
                    fw << "notOFFENSIVESTRAT/notCLOSESTTEAM/SECONDCLOSEST" << "\n";
                }
            }

            
            VecPosition midgoalball = ball.Midpoint(VecPosition(-HALF_FIELD_X, 0, 0));
            VecPosition midmidball = ball.Midpoint(midgoalball);
            VecPosition midmidmidball = ball.Midpoint(midmidball);
            midmidmidball = ValidateFieldPos(midmidmidball);

            if(visualise){
                worldModel->getRVSender()->drawCircle("2nd close",_myPos.getX(), _myPos.getY(),0.7,1,1,0); //Yellow
                worldModel->getRVSender()->drawLine("2nd close line",_myPos.getX(),_myPos.getY(),midmidmidball.getX(), midmidmidball.getY(),1,1,0); //Yellow
            }
            
            float distance = _myPos.getDistanceTo(midmidmidball);
            if(distance < 0.5){
                    if(visualise){
                        worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                    }

                    if(log){
                        if (fw.is_open())
                        {
                            fw << "RETURN FACE BALL" << "\n";
                        }
                    }

                    return FaceBall(_myPos);
                }
            else{

                if(log){
                    if (fw.is_open())
                    {
                        fw << "RETURN GO TO TARGET: " + midmidmidball.toString() << "\n";
                    }
                }
                

                return SmartGoToTarget(midmidmidball,1);
            }

        }
        else{

            if(log){
                if (fw.is_open())
                {
                    fw << "notOFFENSIVESTRAT/notCLOSESTTEAM/notSECONDCLOSEST" << "\n";
                }
            }

            float distance = _myPos.getDistanceTo(mydesiredposition);
            if(distance < 0.5){
                if(visualise){
                    worldModel->getRVSender()->drawCircle("Im at Pos",_myPos.getX(), _myPos.getY(),0.2,1,0.5,0); //ORANGE
                }

                if(log){
                    if (fw.is_open())
                    {
                        fw << "RETURN FACE BALL" << "\n";
                    }
                }
                return FaceBall(_myPos);
            }
            else{

                if(log){
                    if (fw.is_open())
                    {
                        fw << "RETURN GO TO TARGET: " + mydesiredposition.toString() << "\n";
                    }
                }

                if(visualise){
                    worldModel->getRVSender()->drawCircle("Im at Pos",mydesiredposition.getX(), mydesiredposition.getY(),0.2,1,0.5,0); //ORANGE
                }

                return SmartGoToTarget(mydesiredposition,1);
            } 

            
        }

    }

    fw.close();

    return SKILL_STAND;
}


bool NaoBehavior::isTargetDirectlyBehindMe(VecPosition _mypos, VecPosition target){
   
    SIM::Point2D origin = SIM::Point2D(_mypos.getX(),_mypos.getY());



    SIM::AngDeg myCurrAngDegrees = 180-worldModel->getMyAngDeg();
    SIM::Point2D newmiddlepoint = origin.ExtendPointbyAngle(origin,myCurrAngDegrees);

    double offset = 15; //offset in degrees

    SIM::Point2D newmiddleUPpoint = origin.ExtendPointbyAngle(origin,myCurrAngDegrees+offset);
    SIM::Point2D newmiddleDOWNpoint = origin.ExtendPointbyAngle(origin,myCurrAngDegrees-offset);

    bool isInside = origin.isPointInsideTriangle(origin, newmiddleUPpoint, newmiddleDOWNpoint, SIM::Point2D(target.getX(), target.getY()));

    worldModel->getRVSender()->drawLine("newmiddlepoint",origin.x,origin.y,newmiddlepoint.x,newmiddlepoint.y,1,1,1);
    worldModel->getRVSender()->drawLine("Triangulation",newmiddlepoint.x,newmiddlepoint.y,newmiddlepoint.x,newmiddlepoint.y,1,1,1);
    worldModel->getRVSender()->drawLine("newmiddleUPpoint",origin.x,origin.y,newmiddleUPpoint.x,newmiddleUPpoint.y,1,1,1);
    worldModel->getRVSender()->drawLine("newmiddleDOWNpoint",origin.x,origin.y,newmiddleDOWNpoint.x,newmiddleDOWNpoint.y,1,1,1);

    return isInside;
}


bool NaoBehavior::isPassReciever(vector<pair<double,int > > TeamDistToBall){

    double PlayerInPossesionDist = TeamDistToBall[0].first;
     
    if (PlayerInPossesionDist < 0.44){
        
    }
    else{
        return false;
    }
}

int NaoBehavior::DetermineValidPassIDPlayerCentric(vector<pair<double,int > > TeamDistToOppGoal,vector<pair<double,int > > TeamDistToBall, VecPosition playerPos, int kickingDistanceThreshold){


   // cout << "KICKTHRESH"<<kickingDistanceThreshold << endl;

    for (int i = 0; i < TeamDistToOppGoal.size(); i++) {

        //if(myNum != TeamDistToOppGoal[i].second){

            playerPos = worldModel->getWorldObject(TeamDistToOppGoal[i].second)->pos;
            int unum = TeamDistToOppGoal[i].second;
            int indexInTeamDistToBall = -1;

            for (int j = 0; j < TeamDistToBall.size(); j++) {
                if (unum == TeamDistToBall[j].second){
                    indexInTeamDistToBall = j;
                    //cout << "Yes" << endl;
                }
            }

            double dist = TeamDistToBall[indexInTeamDistToBall].first;
            //cout << "Partner NUM " << TeamDistToOppGoal[i].first << "DIST " << dist << " PP " << playerPos.toString() << endl;  
            //string text =  "Partner NUM " + std::to_string(unum) + "D2G " +  std::to_string(TeamDistToOppGoal[i].first) + "D2B " +  std::to_string(dist) + " PP " +  playerPos.toString();
            
            //worldModel->getRVSender()->drawText("distTEAM",text,playerPos.getX(),playerPos.getY()+2,1,1,1);

            
            if(dist<kickingDistanceThreshold){
            // cout << "TARGETPLAYERPREOBS" << playerPos.toString() << endl;  
                if(!(isThereObstructionBetweenTargetAndBall(playerPos))){

                //  cout << "TARGETPLAYER" << playerPos.toString() << endl;    
                    return unum;
                }
            }


       // }

    }
    return NULL;

}

VecPosition NaoBehavior::DetermineValidPassPositionPlayerCentric(VecPosition _myPos, vector<pair<double,int > > TeamDistToOppGoal,vector<pair<double,int > > TeamDistToBall, VecPosition playerPos, int kickingDistanceThreshold, int myNum, bool passBackAllowed){


   // cout << "KICKTHRESH"<<kickingDistanceThreshold << endl;

    for (int i = 0; i < TeamDistToOppGoal.size(); i++) {

        if(myNum != TeamDistToOppGoal[i].second){ //check that we dont consider ourselves

            playerPos = worldModel->getWorldObject(TeamDistToOppGoal[i].second)->pos;
            int unum = TeamDistToOppGoal[i].second;
            int indexInTeamDistToBall = -1;

            for (int j = 0; j < TeamDistToBall.size(); j++) {
                if (unum == TeamDistToBall[j].second){
                    indexInTeamDistToBall = j;
                    //cout << "Yes" << endl;
                }
            }

            double dist = TeamDistToBall[indexInTeamDistToBall].first;
            double xdiff = (playerPos.getX() - _myPos.getX());
            //cout << "Partner NUM " << TeamDistToOppGoal[i].first << "DIST " << dist << " PP " << playerPos.toString() << endl;  
            //string text =  "Partner NUM " + std::to_string(unum) + "D2G " +  std::to_string(TeamDistToOppGoal[i].first) + "D2B " +  std::to_string(dist) + " PP " +  playerPos.toString();
            
            //worldModel->getRVSender()->drawText("distTEAM",text,playerPos.getX(),playerPos.getY()+2,1,1,1);

            
            if(dist<kickingDistanceThreshold){
                if(dist>3.5){ // minimum pass distance check
                    if(xdiff > 3){
                        if(!(isThereObstructionBetweenTargetAndBall(playerPos, 1, 0.5))){
                            if(!(isOpponentWithinRegion(playerPos,1))){

                                if(passBackAllowed){
                                    return playerPos;
                                }
                                else{
                                    if(playerPos.getX() < ball.getX()){

                                    }
                                    else{
                                        return playerPos;
                                    }
                                }
                                
                            }
                        }
                    }
                }
            }
        }

    }
    return NULL;

}

VecPosition NaoBehavior::DetermineGoodKickToLocation(int _playernumber, VecPosition _myPos, double minKickDistThreshold){
    double minX = -15.0, maxX = 15.0;
    double minY = -10.0, maxY = 10.0;
    double incrementX = 0.5, incrementY = 0.5;
    double subRangeMinX = _myPos.getX()-8.0, subRangeMaxX = _myPos.getX() + 8.0;

    if(subRangeMinX < -12.0){
        subRangeMinX = -12.0;
    }

    if(subRangeMaxX > 14.0){
        subRangeMaxX = 14.0;
    }


    double subRangeMinY = -8.0, subRangeMaxY = 8.0;

    vector<pair<double, VecPosition>> KickLocations;

    for (double x = subRangeMinX; x <= subRangeMaxX; x += incrementX) {
        for (double y = subRangeMinY; y <= subRangeMaxY; y += incrementY) {

            VecPosition candidatePos = VecPosition(x,y,0);
            
            if (visualise){
                worldModel->getRVSender()->drawPoint("p", candidatePos.getX(), candidatePos.getY(), RVSender::ORANGE);
            }


            vector<pair<double,int > > teamToCandidatePos = GenerateTeamToTargetDistanceVector(_playernumber, candidatePos);
            vector<pair<double,int > > oppToCandidatePos = GenerateOppToTargetDistanceVector(candidatePos);

            double DistToBall = VecPosition(HALF_FIELD_X,0,0).getDistanceTo(candidatePos);

            double minOppDis = oppToCandidatePos[0].first; 
            double minTeamDis = teamToCandidatePos[0].first; 

            if(minTeamDis < minOppDis){

                if(_myPos.getDistanceTo(candidatePos) > minKickDistThreshold){
                    if (visualise){
                        worldModel->getRVSender()->drawCircle("p", candidatePos.getX(), candidatePos.getY(),0.2, RVSender::ORANGE);
                    }
                    KickLocations.emplace_back(DistToBall,candidatePos);
                }
                
                
            }
        }
    }

    // cout << "PNUM:" << _playernumber << " kicksize: " << KickLocations.size() << endl;

    if(KickLocations.size() != 0){
        sort(KickLocations.begin(), KickLocations.end(), 
                            [](const pair<double, VecPosition> & a, const pair<double, VecPosition> & b) -> bool
                        { 
                            return a.first < b.first; 
                        });


        for (int i = 0; i < KickLocations.size(); i++) {
            auto A = VecPosition(KickLocations[i].second.getX(),KickLocations[i].second.getY(),KickLocations[i].second.getZ());
            
            if(!(isThereObstructionBetweenTargetAndBall(A))){
                return A;
            }
        } 
    }
    else{
        return VecPosition(HALF_FIELD_X,0,0);
    }
    return VecPosition(HALF_FIELD_X,0,0);

}

VecPosition NaoBehavior::DetermineValidPassPositionPlayerCentricFAT(vector<pair<double,int > > TeamDistToOppGoal,vector<pair<double,int > > TeamDistToBall, VecPosition playerPos, int kickingDistanceThreshold, int myNum, bool passBackAllowed){


   // cout << "KICKTHRESH"<<kickingDistanceThreshold << endl;

    for (int i = 0; i < TeamDistToOppGoal.size(); i++) {

        if(myNum != TeamDistToOppGoal[i].second){ //check that we dont consider ourselves

            playerPos = worldModel->getWorldObject(TeamDistToOppGoal[i].second)->pos;
            int unum = TeamDistToOppGoal[i].second;
            int indexInTeamDistToBall = -1;

            for (int j = 0; j < TeamDistToBall.size(); j++) {
                if (unum == TeamDistToBall[j].second){
                    indexInTeamDistToBall = j;
                    //cout << "Yes" << endl;
                }
            }

            double dist = TeamDistToBall[indexInTeamDistToBall].first;
            //cout << "Partner NUM " << TeamDistToOppGoal[i].first << "DIST " << dist << " PP " << playerPos.toString() << endl;  
            //string text =  "Partner NUM " + std::to_string(unum) + "D2G " +  std::to_string(TeamDistToOppGoal[i].first) + "D2B " +  std::to_string(dist) + " PP " +  playerPos.toString();
            
            //worldModel->getRVSender()->drawText("distTEAM",text,playerPos.getX(),playerPos.getY()+2,1,1,1);

            
            if(dist<kickingDistanceThreshold){
                if(dist>3.5){ // minimum pass distance check
                    // cout << "TARGETPLAYERPREOBS" << playerPos.toString() << endl;  
                    if(!(isThereObstructionBetweenTargetAndBall(playerPos))){
                        if(!(isOpponentWithinRegion(playerPos,1))){
                            if(passBackAllowed){
                                return playerPos;
                            }
                            else{
                                if(playerPos.getX() < ball.getX()){

                                }
                                else{
                                    return playerPos;
                                }
                            }


                            
                        }
                    }
                }
            }


        }

    }
    return NULL;

}

bool NaoBehavior::isOpponentWithinRegion(VecPosition playerPos, double threshold){
    for(int i = WO_OPPONENT1; i<WO_OPPONENT1+NUM_AGENTS;i++){ //OPP PLAYERS
        WorldObject* opponent = worldModel->getWorldObject(i);
        VecPosition temp;
        temp = opponent->pos;
        temp.setZ(0);
        if(isInNeighbourhood(playerPos, temp, threshold)){
            return true;
        }
    }
    return false;
}


bool NaoBehavior::AmITheNthClosestTeammateToBall(vector<pair<double,int > > TeamDistToBall, int _playerNumber, int position){
    if(TeamDistToBall[position].second == _playerNumber){
        return true;
    }
    return false;
}

bool NaoBehavior::isClosestPlayertoBall(int _playerNumber, vector<pair<double,int > > TeamDistToBall, vector<pair<double,int > > OppDistToBall){

    double minOppDis = OppDistToBall[0].first; 
    double minTeamDis = TeamDistToBall[0].first; 

    if(minOppDis+1 < minTeamDis){
        return false;
    }
    else{
        if(TeamDistToBall[0].second != _playerNumber){
            return false;
        }
        else{
            return true;
        }
    }
}

bool NaoBehavior::isPossesion(vector<pair<double,int > > TeamDistToBall, vector<pair<double,int > > OppDistToBall){

    double minOppDis = OppDistToBall[0].first; 
    double minTeamDis = TeamDistToBall[0].first; 

    if(minOppDis < (minTeamDis*0.8)){   // this is important
        return false;
    }
    return true;
}


bool NaoBehavior::isClosestTeam(int _playerNumber, vector<pair<double,int > > TeamDistToBall){

    if(_playerNumber == TeamDistToBall[0].second){
        j["isClosestTeam"] = true;
        return true;
    }
    j["isClosestTeam"] = false;
    return false;

}

SkillType NaoBehavior::DetermineAppropriateRLKick(VecPosition _mypos, vector<pair<double,int > > OppDistToBall, VecPosition target){
    if (visualise){
        worldModel->getRVSender()->drawCircle("2nd close",target.getX(), target.getY(),0.7,1,1,0); //Yellow
        worldModel->getRVSender()->drawLine("2nd close line",_mypos.getX(),_mypos.getY(),target.getX(),target.getY(),1,1,0); //Yellow
    }
            
    
    
    double minOppDis = OppDistToBall[0].first;
    int ClosestOppId = OppDistToBall[0].second;
    VecPosition ClosestOppPos = worldModel->getWorldObject(ClosestOppId)->pos;
    double targetDistance = target.getDistanceTo(VecPosition(HALF_FIELD_X,0,0));
    // return doRLKick(target, "normal");
    // return doRLKick(target, "long");
     // testing
    // if (ball.getX() >= -5){
    //     return doRLKick(target, "long");
    // } else {
    //     return doRLKick(target, "normal");
    // }
    if (minOppDis < 1){ // 0.5 is a bit too much
        return doRLKick(target, "fast_no_pid");
    }
    if(targetDistance >= 6){
        
        if (worldModel->getPlayMode() == PM_PASS_LEFT && worldModel->getSide() == SIDE_LEFT
            || worldModel->getPlayMode() == PM_PASS_RIGHT && worldModel->getSide() == SIDE_RIGHT){
            if(minOppDis > 2){
                return doRLKick(target, "long");
            }
        }

        if(minOppDis > 3){ // maybe conservative
            return doRLKick(target, "long");
        } else{
            return doRLKick(target, "normal");
        }
    }
    else{
        return doRLKick(target, "normal");
    }
}


SkillType NaoBehavior::DetermineAppropriateKick(VecPosition _mypos, vector<pair<double,int > > OppDistToBall, VecPosition target){
    double minOppDis = OppDistToBall[0].first;
    int ClosestOppId = OppDistToBall[0].second;
    VecPosition ClosestOppPos = worldModel->getWorldObject(ClosestOppId)->pos;

    if(minOppDis > 1.2){
        if(visualise){
            worldModel->getRVSender()->drawCircle("KICK",target.getX(), target.getY(),0.1,0,0.5,1); //blue
        }
        
        return doRLKick(target); // Mike 2023/07/02
        return kickBall(KICK_IK, target);
    }
    else if(minOppDis < 0.5){
        if(visualise){
            worldModel->getRVSender()->drawCircle("KICK",target.getX(), target.getY(),0.1,0,0,1); //dark blue
        }
        
        return kickBall(KICK_DRIBBLE, target);
    }
    else{
        if(visualise){
            worldModel->getRVSender()->drawCircle("KICK",target.getX(), target.getY(),0.1,0,0.5,1); //blue
        }
        
        return doRLKick(target); // Mike 2023/07/02
        return kickBall(KICK_IK, target);
    }


    // if(minOppDis <0.5){
    //     //worldModel->getRVSender()->drawCircle("KICK",target.getX(), target.getY(),0.1,0,0.5,1); //blue
    //     return kickBall(KICK_IK, target);
    // }
    // else if(minOppDis >4.2){
    //     //worldModel->getRVSender()->drawCircle("KICK",target.getX(), target.getY(),0.1,0,1,1); //light blue
    //     return kickBall(KICK_FORWARD, target);
    // }
    // else{
    //     if(ClosestOppPos.getX() > _mypos.getX()){
    //         //worldModel->getRVSender()->drawCircle("KICK",target.getX(), target.getY(),0.1,0,0.5,1); //blue
    //         return kickBall(KICK_IK, target);
    //     }
    //     else{
    //         //worldModel->getRVSender()->drawCircle("KICK",target.getX(), target.getY(),0.1,0,0,1); //dark blue
    //         return kickBall(KICK_DRIBBLE, target);
    //     }
        
    // }
}

bool NaoBehavior::isSafeToKick(vector<pair<double,int > > OppDistToBall){
    double minOppDis = OppDistToBall[0].first;
    if(minOppDis <1){
        return false;
    }
    return true;

}

bool NaoBehavior::BallinMyGoalMouth(){
    double distanceToGoal = worldModel->distanceToMyGoal(ball);
    if(distanceToGoal < 3){
        return true;
    }
    return false;
}


bool NaoBehavior::BallinOppGoalMouth(){
    double distanceToGoal = worldModel->distanceToOppGoal(ball);
    if(distanceToGoal < 2){

        if((ball.getY() > -HALF_GOAL_Y) && (ball.getY() < HALF_GOAL_Y)){
            return true;
        }

        return false;
    }
    return false;
}

bool NaoBehavior::isShootingRange(VecPosition _player, double range){
    
    double distanceToGoal = worldModel->distanceToOppGoal(_player);

    if(distanceToGoal < range){
        return true;
    }
    return false;

}

VecPosition NaoBehavior::getClosestTeammatePos(int _playerNumber,vector<pair<double,int > > TeamDistToBall){
    
    // cout << "Player Closest To Ball \n";
    // for (int i = 0; i < TeamDistToBall.size(); i++) {
    //     cout << "("
    //         << TeamDistToBall[i].first << ", "
    //         << TeamDistToBall[i].second << ") ";
    // }
    // cout << "\n"; 

    // cout << _playerNumber << "\n";


    int i=0;
    while(TeamDistToBall[i].second == _playerNumber){
        i++;
        //cout << TeamDistToBall[i].second << "\n";
    }
    //cout << "X=" << worldModel->getWorldObject(TeamDistToBall[i].second)->pos.getX() << "\n";

    return worldModel->getWorldObject(TeamDistToBall[i].second)->pos;
    
}

VecPosition NaoBehavior::ValidateFieldPos(VecPosition target){
    if(target.getX() <= (-1*FIELD_X)/2){
        target.setX((-1*FIELD_X)/2-0.1); 
    }
    else if(target.getX() >= FIELD_X/2){
        target.setX(FIELD_X/2 +0.1); 
    }

    if(target.getY() <= (-1*FIELD_Y)/2){
        target.setY((-1*FIELD_Y)/2-0.1);
    }
    else if(target.getY() >= FIELD_Y/2){
        target.setY(FIELD_Y/2 +0.1);
    }

    return target;
}

SkillType NaoBehavior::SmartGoToTargetWithPrecisionAndFaceSpecificLocation(VecPosition target, double proximity,double localprecision, VecPosition _mypos, VecPosition FaceLocation){

    JSONLOG_StrategyDataToLog("GOTO_Face",target);
    SIM::AngDeg localAngle = atan2Deg(ball.getY(), ball.getX());

    // Adjust target to not be too close to teammates or the ball
        target = collisionAvoidance(true /*teammate*/, true/*opponent*/, false/*ball*/, proximity/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);

        if (me.getDistanceTo(target) < localprecision && abs(localAngle) <= 90) {
            // Close enough to desired position and orientation so just stand
            return FaceTarget(_mypos, FaceLocation);
        } else if (me.getDistanceTo(target) < localprecision) {
            // Close to desired position so start turning to face ball
            return goToTargetRelative(worldModel->g2l(target), localAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
}

SkillType NaoBehavior::SmartGoToTargetWithPrecision(VecPosition target, double proximity,double localprecision){

    JSONLOG_StrategyDataToLog("FATGOTO",target);
    SIM::AngDeg localAngle = atan2Deg(ball.getY(), ball.getX());

    // Adjust target to not be too close to teammates or the ball
        target = collisionAvoidance(true /*teammate*/, true/*opponent*/, false/*ball*/, proximity/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);

        if (me.getDistanceTo(target) < localprecision && abs(localAngle) <= 90) {
            // Close enough to desired position and orientation so just stand
            return SKILL_STAND;
        } else if (me.getDistanceTo(target) < localprecision) {
            // Close to desired position so start turning to face ball
            return goToTargetRelative(worldModel->g2l(target), localAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
}

SkillType NaoBehavior::SmartGoToTarget(VecPosition target, double proximity){

    JSONLOG_StrategyDataToLog("GOTO",target);
    SIM::AngDeg localAngle = atan2Deg(ball.getY(), ball.getX());

    // Adjust target to not be too close to teammates or the ball
        target = collisionAvoidance(true /*teammate*/, true/*opponent*/, false/*ball*/, proximity/*proximity thresh*/, .5/*collision thresh*/, target, true/*keepDistance*/);

        if (me.getDistanceTo(target) < .5 && abs(localAngle) <= 90) {
            // Close enough to desired position and orientation so just stand
            return SKILL_STAND;
        } else if (me.getDistanceTo(target) < .5) {
            // Close to desired position so start turning to face ball
            return goToTargetRelative(worldModel->g2l(target), localAngle);
        } else {
            // Move toward target location
            return goToTarget(target);
        }
}


bool NaoBehavior::atDestination(VecPosition mypos, VecPosition target){
    double distance =  mypos.getDistanceTo(target);

    if(distance <0.4){
        return true;
    }
    return false;
}


SkillType NaoBehavior::FaceBall(VecPosition mypos){

    JSONLOG_StrategyDataToLog("SKILL_STAND",mypos);
    double distance, angle;
    getTargetDistanceAndAngle(ball, distance, angle);
    if (abs(angle) > 30) {
      return goToTargetRelative(mypos, angle);
    } else {
      return SKILL_STAND;
    }
}

SkillType NaoBehavior::FaceTarget(VecPosition mypos, VecPosition target){

    double distance, angle;
    getTargetDistanceAndAngle(target, distance, angle);
    if (abs(angle) > 10) {
      return goToTargetRelative(mypos, angle);
    } else {
      return SKILL_STAND;
    }
}



bool NaoBehavior::DangerClose(vector<pair<double,int > > OppDistToBall){
    double minOppDis = OppDistToBall[0].first; 
    int id = OppDistToBall[0].second; 
    id = id + WO_OPPONENT1;    

    if(minOppDis<1){

        WorldObject* closestopponent = worldModel->getWorldObject(id);
        VecPosition closestopponentpos = closestopponent->pos;

        double distogoal1 = closestopponentpos.getDistanceTo(VecPosition(-HALF_FIELD_X, 0, 0));
        double distogoal2 = ball.getDistanceTo(VecPosition(-HALF_FIELD_X, 0, 0));

        if(distogoal1>distogoal2){ //opponent on the right side of the ball to be a threat
            return true;
        }
        else{
            return false;
        }
    }
    else{
        return false;
    }

}
/*
 * Centre Back behavior tries to intercept player within a specific distance to the goal
 */
SkillType NaoBehavior::centreBackBehaviour() {
    SIM::Point2D ball2Point2D = SIM::Point2D(ball.getX(), ball.getY());
    SIM::Point2D goalLine = SIM::Point2D(-HALF_FIELD_X, 0);
    SIM::Point2D opp1 = SIM::Point2D(worldModel->getOpponent(WO_OPPONENT1).getX(),worldModel->getOpponent(WO_OPPONENT1).getY());

    SIM::Point2D pos2 = opp1.getPointOnLineFraction(goalLine, 0.1);

    // worldModel->getRVSender()->drawAgentText("ball " + to_string(pos2.getX()) + to_string(pos2.getY()));
    //worldModel->getRVSender()->drawPoint("line", pos2.getX(), pos2.getY(), RVSender::GREEN);
    // worldModel->getRVSender()->drawPoint("opp1", opp1.getX(), opp1.getY(), RVSender::BLUE);
    
    SIM::AngDeg localCenterAngle = goalLine.getAngleTo(pos2);
    VecPosition target = VecPosition(pos2.getX(), pos2.getY(), 0);
    target = collisionAvoidance(true /*teammate*/, true/*opponent*/, false/*ball*/, 1/*proximity thresh*/, .3/*collision thresh*/, target, false/*keepDistance*/);
    //worldModel->getRVSender()->drawPoint("Point", target.getX(), target.getY(), RVSender::GREEN);
    if (me.getDistanceTo(target) < .25 && abs(localCenterAngle) <= 10) {
        // Close enough to desired position and orientation so just stand
        return SKILL_STAND;
    }
    else {
        // Move toward target location
        return goToTarget(target);
    }
}

bool NaoBehavior::isOffensiveStrat(int pNum, vector<pair<double,int > > TeamDistToBall, vector<pair<double,int > > OppDistToBall){
    //return isPossesion(_opponentDistances, _teamMateDistances);

    if(isPossesion(TeamDistToBall, OppDistToBall)){

        if(isClosestTeam(pNum, TeamDistToBall)){

            j["IsOffensive"] = true;
            return true;
        }
        else{
            if(DangerClose(OppDistToBall)){
                j["IsOffensive"] = false;
                return false;
            }
            else{
                j["IsOffensive"] = true;
                return true;
            }
        }


    }
    else{
        j["IsOffensive"] = false;
        return false;
    }

}

int NaoBehavior::getXFieldRegion(VecPosition objpos){
    int x = objpos.getX();


    // worldModel->getRVSender()->drawLine("REGIONX5",-5, -10, -5, 10,0,0,0);
    // worldModel->getRVSender()->drawLine("REGIONX0",0, -10, 0, 10,0,0,0);
    // worldModel->getRVSender()->drawLine("REGIONX-5",5, -10, 5, 10,0,0,0);
    // worldModel->getRVSender()->drawText("0,0","0,0",0,0,0,0,0);
    // worldModel->getRVSender()->drawText("5,0","5,0",5,0,0,0,0);
    // worldModel->getRVSender()->drawText("-5,0","-5,0",-5,0,0,0,0);
    // worldModel->getRVSender()->drawText("-5,10","-5,10",-5,10,0,0,0);
    // worldModel->getRVSender()->drawText("-5,-10","-5,-10",-5,-10,0,0,0);
    // worldModel->getRVSender()->drawText("0,-10","0,-10",0,-10,0,0,0);
    // worldModel->getRVSender()->drawText("0,10","0,10",0,10,0,0,0);
    // worldModel->getRVSender()->drawText("5,-10","5,-10",5,-10,0,0,0);
    // worldModel->getRVSender()->drawText("5,10","5,10",5,10,0,0,0);
    // worldModel->getRVSender()->drawText("0,-5","0,-5",0,-5,0,0,0);
    
    if(x <= -5){
        return 1;
    }
    else if(x <= 0 && x > -5){
        return 2;
    }
    else if(x<5 && x > 0){
        return 3;
    }
    else{
        return 4;
    }
}

int NaoBehavior::getYFieldRegion(VecPosition objpos){
    int y = objpos.getY();
    

    
    // worldModel->getRVSender()->drawLine("REGIONY-5",-15, -5, 15, -5,0,0,0);
    // worldModel->getRVSender()->drawLine("REGIONY5",-15, 5, 15, 5,0,0,0);
    // worldModel->getRVSender()->drawCircle("GOALMOUTH", -15, 0,3,0,0,0); 
    // worldModel->getRVSender()->drawText("15,5","15,5",15,5,0,0,0);
    // worldModel->getRVSender()->drawText("15,-5","15,-5",15,-5,0,0,0);
    // worldModel->getRVSender()->drawText("-15,5","-15,5",-15,5,0,0,0);
    // worldModel->getRVSender()->drawText("-15,-5","-15,-5",-15,-5,0,0,0);

    if(y>5){
        return 1;
    }
    else if(y<=5 && y> -5){
        return 2;
    }
    else{
        return 3;
    }
    
}

bool NaoBehavior::isInNeighbourhood(VecPosition obj, VecPosition target, double threshold){

    double distance =  obj.getDistanceTo(target);
    if(distance < threshold){
        return true;
    }
    return false;
}

bool NaoBehavior::isAnyTeammateInNeighbourhood(int pNum, VecPosition target, double threshold){

    for(int i = WO_TEAMMATE1; i<WO_TEAMMATE1+NUM_AGENTS;i++){ //OUR PLAYERS
        WorldObject* teammate = worldModel->getWorldObject(i);
        VecPosition temp;

        if(i == pNum){
            temp = worldModel->getMyPosition();
        }
        else{
            temp = teammate->pos;
        }
        temp.setZ(0);

        float distance = temp.getDistanceTo(target);
        if(distance < threshold){
            return true;
        }
    }
    return false;
}

/*
 * OnBall behavior tries to drive ball to the goal
 */
SkillType NaoBehavior::onBallBehaviour() {
    VecPosition target = VecPosition(HALF_FIELD_X+1, 0, 0);
    VecPosition myPos = VecPosition(worldModel->getMyPosition().getX(), worldModel->getMyPosition().getY());
    // target = collisionAvoidance(true /*teammate*/, true/*opponent*/, false/*ball*/, 5/*proximity thresh*/, 5/*collision thresh*/, target, true/*keepDistance*/);
    if(improperPlayMode()){
        return SKILL_STAND;
    }
    else if(worldModel->getPlayMode() != PM_PLAY_ON) {
        return kickBall(KICK_FORWARD, target);
    }
    else if(worldModel->distanceToOppGoal(myPos) < 5) {
        return kickBall(KICK_IK, target);
    } else {
        return kickBall(KICK_DRIBBLE, target);
    }
}


int NaoBehavior::playerClosestToBall() {
    // Find closest player to ball
    int playerClosestToBall = -1;
    double closestDistanceToBall = 10000;
    for(int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i) {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() == playerNum) {
            // This is us
            temp = worldModel->getMyPosition();
        } else {
            WorldObject* teammate = worldModel->getWorldObject( i );
            if (teammate->validPosition) {
                temp = teammate->pos;
            } else {
                continue;
            }
        }
        temp.setZ(0);

        double distanceToBall = temp.getDistanceTo(ball);
        if (distanceToBall < closestDistanceToBall) {
            playerClosestToBall = playerNum;
            closestDistanceToBall = distanceToBall;
        }
    }
    return playerClosestToBall;
}

VecPosition NaoBehavior::GetDesiredTargetAdvanced(int pNum, VecPosition mypos, int regionX, int regionY, vector<pair<double,int > > TeamDistToBall){

    if(regionX ==1){
        double ballX = ball.getX();

        if(regionY ==1){
            return VecPosition(ballX+5.5, 4, 0);
        }
        else if(regionY ==2){
                //this is the main block containing our goal
            
            

            if(mypos.getY() > ball.getY()){
                ///play downwards
                return VecPosition(ballX+5.5, -4, 0);
            }
            else{
                ///play upwards
                return VecPosition(ballX+5.5, 4, 0);
            }
        }
        else{
            return VecPosition(ballX+5.5, -4, 0);
        }
    }
    else if(regionX ==2){

        double ballX = ball.getX();

        float dist = VecPosition(0,0,0).getDistanceTo(ball);
        if(dist < 0.2){
            return VecPosition(ballX+5.5, 4, 0);
        }

        if(ball.getY() > 0){
            return VecPosition(ballX+5.5, 4, 0);
        }
        else{
            return VecPosition(ballX+5.5, -4, 0);
        } 
    }
    else if(regionX ==3){


        VecPosition partner = getClosestTeammatePos(pNum,TeamDistToBall);
        //worldModel->getRVSender()->drawCircle("PASS Partner x3",partner.getX(), partner.getY(),0.7,1,1,0); //Yellow

        float dist = VecPosition(5,10,0).getDistanceTo(ball);
        if(dist < 0.2){
            return VecPosition(8, 3, 0);
        }

        if(ball.getY() > 0){
            return VecPosition(8, 3, 0);
        }
        else{
            return VecPosition(8, -3, 0);
        }

    }
    else{
        VecPosition partner = getClosestTeammatePos(pNum,TeamDistToBall);
        //worldModel->getRVSender()->drawCircle("PASS Partner x4",partner.getX(), partner.getY(),0.7,1,1,0); //Yellow

        if(partner.getX()> mypos.getX()+2 && mypos.getDistanceTo(partner) < 5){
            return VecPosition(partner.getX()+1, partner.getY(), 0);
        }

        if(ball.getY() > 0){
            return VecPosition(HALF_FIELD_X+0.3, -0.5, 0);
        }
        else{
            return VecPosition(HALF_FIELD_X+0.3, 0.5, 0);
        }
    }

    return mypos;
}

bool NaoBehavior::isBetweenTargetAndBall(VecPosition target, VecPosition mypos, double threshold){

    double distanceballtarget = ball.getDistanceTo(target);
    double distanceballmetarget = ball.getDistanceTo(mypos) + mypos.getDistanceTo(target);



    
    double difference = abs(distanceballtarget - distanceballmetarget);
    if(difference < threshold){
        return true;
    }
    return false;
/*
    if (distance(A, C) + distance(B, C) == distance(A, B))
        return true; // C is on the line.
    return false;    // C is not on the line.

    or just:

    return distance(A, C) + distance(B, C) == distance(A, B);

    The way this works is rather simple. If C lies on the AB line, you'll get the following scenario:

    A-C------B

    and, regardless of where it lies on that line, dist(AC) + dist(CB) == dist(AB). For any other case, you have a triangle of some description and 'dist(AC) + dist(CB) > dist(AB)':

    A-----B
     \   /
      \ /
       C
*/

}

vector<pair<double,int > > NaoBehavior::GenerateTeamToTargetDistanceVector(int _playernumber, VecPosition target){

    target.setZ(0);
    vector<pair<double,int> > distances;

    for(int i = WO_TEAMMATE1; i<WO_TEAMMATE1+NUM_AGENTS;i++){ //OUR PLAYERS
        WorldObject* teammate = worldModel->getWorldObject(i);
        VecPosition temp;

        if(i == _playernumber){
            temp = worldModel->getMyPosition();
        }
        else{
            temp = teammate->pos;
        }
        temp.setZ(0);

        

        float distance = temp.getDistanceTo(target);

        if(worldModel->getFallenTeammate(i-1)){
            distance = distance+3;
        }

        if(ball.getX()<(temp.getX()-0.5)){
            // distance = distance+0; //removed penalty from being on wrong side of ball
            if (fFatProxy){
                distance = distance+0;
            }else{
                distance = distance+2;
            }
        }

        pair<double, int> temppoint = {distance,i};
        distances.push_back(temppoint);


        //worldModel->getRVSender()->drawText("distTEAM",std::to_string(distance),temp.getX(),temp.getY()+2*mynum,1,1,1);

    }

    // if(_playernumber ==1){
    //     cout << "UNSORTED \n";
    //     for (int i = 0; i < distances.size(); i++) {
    //         cout << "("
    //             << distances[i].first << ", "
    //             << distances[i].second << ") ";
    //     }
    //     cout << "\n"; 
    // }
    sort(distances.begin(), distances.end());

    // if(_playernumber ==1){
    //     cout << "SORTED \n";
    //     for (int i = 0; i < distances.size(); i++) {
    //         cout << "("
    //             << distances[i].first << ", "
    //             << distances[i].second << ") ";
    //     }
    //     cout << "\n"; 
    // }

    return distances;


}

vector<pair<double,int > > NaoBehavior::GenerateOppToTargetDistanceVector(VecPosition target){

    vector<pair<double,int> > distances;
    target.setZ(0);

    for(int i = WO_OPPONENT1; i<WO_OPPONENT1+NUM_AGENTS;i++){ //OPP PLAYERS
        WorldObject* opponent = worldModel->getWorldObject(i);
        VecPosition temp;
        temp = opponent->pos;
        temp.setZ(0);

        float distance = temp.getDistanceTo(target);

        if(worldModel->getFallenOpponent(i-WO_OPPONENT1)){
            distance = distance+5;
        }

        if(ball.getX()>temp.getX()){
            distance = distance+2;
        }


        pair<double, int> temppoint = {distance,i};
        distances.push_back(temppoint);


        //worldModel->getRVSender()->drawText("distTEAM",std::to_string(distance),temp.getX(),temp.getY()+2*mynum,1,1,1);

    }

    sort(distances.begin(), distances.end());
    /* for (int i = 0; i < distances.size(); i++) {
        cout << "("
             << distances[i].first << ", "
             << distances[i].second << ") ";
    }
    cout << "\n"; */

    return distances;

}

vector<vector<pair<double,int >>> NaoBehavior::GeneratePreferenceArrayForTeam(int _playerNumber, vector<VecPosition> ImportantPositions){

    vector<vector<pair<double,int > > > PreferenceToPointArray;
    
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
        PreferenceToPointArray.push_back(GeneratePreferenceArrayForAnAgent(ImportantPositions,  temp));
    }
    
    return PreferenceToPointArray;
}




vector<pair<double,int > > NaoBehavior::GeneratePreferenceArrayForAnAgent(vector<VecPosition> ImportantPositions, VecPosition playerpos){
    vector<pair<double, double> > arr;

    for(int i=0;i<NUM_AGENTS;i++){
        pair<double, double> temp = {ImportantPositions[i].getX(),ImportantPositions[i].getY()};
        arr.push_back(temp);
    }
    int n = NUM_AGENTS;
    pair<double, double> p = {playerpos.getX(), playerpos.getY()};

    vector<pair<double,int > > sortedarr;
    sortedarr = sortArr(arr, n, p);
    return sortedarr;
}


// Function to sort the array of
// points by its distance from P
vector<pair<double,int > > NaoBehavior::sortArr(vector<pair<double, double> > arr,int n, pair<double, double> p)
{
  
    // Vector to store the distance
    // with respective elements
    vector<pair<double,int> > vp;
  
    // Storing the distance with its
    // distance in the vector array
    for (int i = 0; i < n; i++) {
  
        double dist
            = pow((p.first - arr[i].first), 2)
              + pow((p.second - arr[i].second), 2);
  
        vp.push_back(make_pair(dist,i));
    }
  
    // Sorting the array with
    // respect to its distance
    sort(vp.begin(), vp.end());
  
    // Output
    /* for (int i = 0; i < vp.size(); i++) {
        cout << "("
             << vp[i].first << ", "
             << vp[i].second << ") ";
    }
    cout << "\n"; */

    return vp;
}


// This function returns true if woman 'w' prefers man 'm1' over man 'm'
bool NaoBehavior::wPrefersM1OverM(int prefer[2*NUM_AGENTS][NUM_AGENTS], int w, int m, int m1)
{
    // Check if w prefers m over her current engagment m1
    for (int i = 0; i < NUM_AGENTS; i++)
    {
        // If m1 comes before m in lisr of w, then w prefers her
        // cirrent engagement, don't do anything
        if (prefer[w][i] == m1)
            return true;
  
        // If m cmes before m1 in w's list, then free her current
        // engagement and engage her with m
        if (prefer[w][i] == m)
           return false;
    }
    return false;
}
  
// Determines stable matching for N Players and N Points. Players are numbered as 0 to
// N-1. Points are numbereed as N to 2N-1.
vector<int> NaoBehavior::stableMarriage(vector<vector<pair<double,int > > > PreferenceToPointArray)
{
    // Convert vector to 2D array
    int prefer[2*NUM_AGENTS][NUM_AGENTS];

    for(int i=0;i<NUM_AGENTS;i++){
        for(int j=0;j<NUM_AGENTS;j++){
            prefer[i][j] = PreferenceToPointArray[i][j].second;
            prefer[i+NUM_AGENTS][j] = j;
        }
    }

    // Stores partner of Points. This is our output array that
    // stores paring information.  The value of wPartner[i]
    // indicates the partner assigned to Point N+i.  Note that
    // the Points number between N and 2*N-1. The value -1
    // indicates that (N+i)'th point is free
    int wPartner[NUM_AGENTS];
  
    // An array to store availability of Players.  If mFree[i] is
    // false, then player 'i' is free, otherwise engaged.
    bool mFree[NUM_AGENTS];
  
    // Initialize all players and points as free
    memset(wPartner, -1, sizeof(wPartner));
    memset(mFree, false, sizeof(mFree));
    int freeCount = NUM_AGENTS;
  
    //for (int p=0;p<NUM_AGENTS;p++){
    //    cout << wPartner[p] << "\n";
    //}


    // While there are free players
    while (freeCount > 0)
    {
        // Pick the first free player (we could pick any)
        int m;
        for (m = 0; m < NUM_AGENTS; m++)
            if (mFree[m] == false)
                break;
  
        // One by one go to all points according to m's preferences.
        // Here m is the picked free player
        for (int i = 0; i < NUM_AGENTS && mFree[m] == false; i++)
        {
            int w = prefer[m][i];

            // The point of preference is free, w and m become
            // partners (Note that the partnership maybe changed
            // later). So we can say they are engaged not married

            if (wPartner[w] == -1)
            {
                wPartner[w] = m;
                mFree[m] = true;
                freeCount--;
            }
            else  // If w is not free
            {
                // Find current engagement of w
                int m1 = wPartner[w];
  
                // If w prefers m over her current engagement m1,
                // then break the engagement between w and m1 and
                // engage m with w.
                if (wPrefersM1OverM(prefer, w, m, m1) == false)
                {
                    wPartner[w] = m;
                    mFree[m] = true;
                    mFree[m1] = false;
                }
            } // End of Else
        } // End of the for loop that goes to all points in m's list
    } // End of main while loop
  
  
    // Print the solution
    /* cout << "Point   Player" << endl;
    for (int i = 0; i < NUM_AGENTS; i++)
       cout << " " << i << "\t" << wPartner[i] << endl; */


    vector<int> v(std::begin(wPartner), std::end(wPartner));
    return v;   
}

double NaoBehavior::MapValueInRangeTo(double input, double input_start, double input_end, double output_start, double output_end){

    double output = output_start + ((output_end - output_start) / (input_end - input_start)) * (input - input_start);
    return output;
}

/**
 *  This function checks if there is any obstruction (be it a teammate or an opponent) between the ball
 *  and the target. Not sure how we're going to make sure the target is not an obstruction?
 * Threshold is the minimum distance a teammate has to be from the target to be classified as an obstruction.
 */
bool NaoBehavior::isThereObstructionBetweenTargetAndBall(VecPosition target, double threshold, double width){
    // todo what about the actual player to kick and the target if it's a player?
    VecPosition vector_to_target = target - ball;
    auto normal = vector_to_target.normalize();
    auto orthogonal1 = VecPosition(-normal.getY(), normal.getX());
    
    auto s1 = ball + orthogonal1*width;
    auto s2 = ball - orthogonal1*width;
    auto e1 = target + orthogonal1*width;
    auto e2 = target - orthogonal1*width;


    SIM::Line2D line1 = SIM::Line2D(SIM::Point2D(s1.getX(), s1.getY()), SIM::Point2D(e1.getX(), e1.getY()));
    SIM::Line2D line2 = SIM::Line2D(SIM::Point2D(s2.getX(), s2.getY()), SIM::Point2D(e2.getX(), e2.getY()));
    
    SIM::Line2D lineStart = SIM::Line2D(
        SIM::Point2D(s1.getX(), s1.getY()),
        SIM::Point2D(s2.getX(), s2.getY())
    );

    SIM::Line2D lineEnd = SIM::Line2D(
        SIM::Point2D(e1.getX(), e1.getY()),
        SIM::Point2D(e2.getX(), e2.getY())
    );

    line1.m_c *=-1;
    line2.m_c *=-1;
    lineStart.m_c *= -1;
    lineEnd.m_c *= -1;
    

    if (line1.m_c > line2.m_c){
        std::swap(line1, line2);
    }
    if (lineStart.m_c > lineEnd.m_c){
        std::swap(lineStart, lineEnd);
    }
    
    
    // DEBUG DRAW
    //if(visualise){
    // worldModel->getRVSender()->drawLine("ObstructionZone",
    //     s1.getX(), s1.getY(),
    //     e1.getX(), e1.getY(), 1, 0, 0
    // );
    // worldModel->getRVSender()->drawLine("ObstructionZone",
    //     s2.getX(), s2.getY(),
    //     e2.getX(), e2.getY(), 0, 0, 1
    // );
    // worldModel->getRVSender()->drawLine("ObstructionZone",
    //     s2.getX(), s2.getY(),
    //     s1.getX(), s1.getY(), 0, 1, 1
    // );
    // worldModel->getRVSender()->drawLine("ObstructionZone",
    //     e2.getX(), e2.getY(),
    //     e1.getX(), e1.getY(), 1, 1, 0
    // );
   // }
    
    // now, the slopes should be the same and just the c's should differ.
    double mx = line1.m_a;
    double my = line1.m_b;
    
    double c1 = line1.m_c;
    double c2 = line2.m_c;

    if (c1 > c2){
        std::swap(c1, c2);
    }

    auto isBetweenLines = [&](VecPosition vec, SIM::Line2D l1, SIM::Line2D l2){
        // l1.c must be less than l2.c
        auto v = vec;// - ball;
        double c_from_vec = v.getX() * l1.m_b + v.getY() * l1.m_a;
        if (l1.m_c <= c_from_vec && c_from_vec <= l2.m_c){
            return true;
        }
        return false;
    };

    // now iterate over all team mates and enemies.
    // todo MIKE.
    for (int start_idx: {WO_TEAMMATE1, WO_OPPONENT1}){ // WO_OPPONENT1, 
        for (int i=start_idx; i < start_idx + NUM_AGENTS; ++i){
            VecPosition pos = worldModel->getWorldObject(i)->pos;
            if (i == worldModel->getUNum()){
                pos = worldModel->getMyPosition();
            }
            if (start_idx == WO_TEAMMATE1){
                // we are looking at a teammate.
                float distance_to_target = pos.getDistanceTo(target);
                if (distance_to_target < threshold){
                    // if this teammate is too close to the target, then don't count it as an obstruction.
                    continue;
                }
            }
            bool l12 = isBetweenLines(pos, line1, line2), lse = isBetweenLines(pos, lineStart, lineEnd);
            if (l12 && lse){
                if(visualise){
                    worldModel->getRVSender()->drawCircle("blocked",pos.getX(), pos.getY(), 2, 1, 1, 1 );
                }
                
                return true;
            }
        }
    }

    return false;
}
VecPosition NaoBehavior::DetermineValidPassRecipient(VecPosition currTarget, VecPosition mypos, int _playerNumber, vector<pair<double,int > > TeamDistToBall, bool passbackallowed){
    if(isThereObstructionBetweenTargetAndBall(currTarget)){
        bool found = false;
        int currIndex = 0;
        while(!found){
            int pNum = TeamDistToBall[currIndex].second;
            if(_playerNumber != pNum){
                VecPosition tempTarget = worldModel->getWorldObject(pNum)->pos;
                if(isThereObstructionBetweenTargetAndBall(tempTarget)){

                }
                else{
                    if(passbackallowed){
                        double distToTarget = mypos.getDistanceTo(tempTarget);
                        if(distToTarget>2){ // prevent very close range pass
                            return tempTarget;
                        }             
                    }
                    else if(tempTarget.getX() > mypos.getX()){ //prevent back pass
                        double distToTarget = mypos.getDistanceTo(tempTarget);
                        if(distToTarget>2){ // prevent very close range pass
                            return tempTarget;
                        }
                    }

                    
                }
            }

            currIndex++;
            if(currIndex>4){ // if we cant find a viable pass from the 4 closest players then move on
                found = true;
            }
        }

        if(ball.getY()>0){
            currTarget.setY(currTarget.getY()-1.5);
            return currTarget;
        }
        else{
            currTarget.setY(currTarget.getY()+1.5);
            return currTarget;
        }


    }
    else{
        return currTarget;
    }
}

VecPosition NaoBehavior::ExtendShotInDirection(VecPosition _myPos, VecPosition _target, double distance){
    //cout << "InsideExetend" << endl;
    VecPosition distVec = _target - _myPos;
    VecPosition adjusted = distVec.normalize()*((distVec.getMagnitude() * distance) + 5);
    adjusted = _myPos + adjusted;
    //cout << "Calculated Adjusted" << endl;
    if(visualise){
        string text = "Original Target:" + _target.toString() + " Adjusted Target:" + adjusted.toString();
        //cout << text << endl;
       // worldModel->getRVSender()->drawText("Targeting",text,0,HALF_FIELD_Y-2,0,0,0);
        worldModel->getRVSender()->drawLine("Shot Extension",
            _target.getX(), _target.getY(),
            adjusted.getX(), adjusted.getY(), 1, 1, 1
        );
        //cout << "After Vis" << endl;
    }
    

    return adjusted; 

}

VecPosition NaoBehavior::DetermineShootAtGoalPositionOld(VecPosition _myPos, int _playerNumber, vector<pair<double,int > > TeamDistToBall){

    double offset = 0.2;
    VecPosition initTarget = VecPosition(HALF_FIELD_X,HALF_GOAL_Y-0.3,0);

    while(isThereObstructionBetweenTargetAndBall(initTarget,1,0.125)){
        initTarget.setY(initTarget.getY()-offset);
        if(initTarget.getY() < (-HALF_GOAL_Y+offset)){
            //double yValue = MapValueInRangeTo(ball.getY(), -HALF_FIELD_Y, HALF_FIELD_Y, -HALF_GOAL_Y, HALF_GOAL_Y);
            //initTarget.setY(yValue);
            VecPosition Adjustedtarget = DetermineValidPassRecipient(initTarget,_myPos,_playerNumber,TeamDistToBall,true);
            Adjustedtarget = ExtendShotInDirection(ball,Adjustedtarget,2.0);

            return Adjustedtarget;
        }
    }
    return ExtendShotInDirection(ball,initTarget,2.0);;
    
}

VecPosition NaoBehavior::DetermineShootAtGoalPosition(VecPosition _myPos, int _playerNumber, vector<pair<double,int > > TeamDistToBall){

    double offset = 0.2;
    VecPosition initTarget = VecPosition(HALF_FIELD_X,HALF_GOAL_Y-0.3,0);
    VecPosition Adjustedtarget = initTarget;

    double maxScore = 0;
    double maxY = 0;

    if(visualise){
        worldModel->getRVSender()->drawLine("goal width", HALF_FIELD_X+0.3,HALF_GOAL_Y-0.3, HALF_FIELD_X+0.3,-HALF_GOAL_Y+0.3);
    }



    for (double y=HALF_GOAL_Y-0.2; y > -HALF_GOAL_Y+0.2; y = y - offset){

        VecPosition initTarget = VecPosition(HALF_FIELD_X,y,0);
        double score = 0;

        double boundaryDistance = min(abs(y - HALF_GOAL_Y), abs(y + HALF_GOAL_Y));

        for (double width = 0.125; width <= min(0.5,boundaryDistance); width = width + 0.125){

            if (isThereObstructionBetweenTargetAndBall(initTarget,1,width)){
                break;
            }
            else{
                score = width;
            }
        }


        if(score > maxScore){
            maxScore = score;
            maxY = y;
        }
        else if(score == maxScore && abs(maxY - _myPos.getY()) > abs(y - _myPos.getY())){  // Tie break is on y distance from the player
            maxScore = score;
            maxY = y;
        }

    }

    Adjustedtarget = VecPosition(HALF_FIELD_X,maxY,0);
    Adjustedtarget = ExtendShotInDirection(ball,Adjustedtarget,2.0);

    return Adjustedtarget;
    
}

void NaoBehavior::DetermineUseGoalieCatchBall(int _playerNumber){
    
    VecPosition pos = worldModel->getWorldObject(WO_TEAMMATE1)->pos;
    if (_playerNumber == worldModel->getUNum()){
        pos = worldModel->getMyPosition();
    }
    pos.setZ(0);
    double GoalieDistanceToBall = pos.getDistanceTo(ball);

    if(_playerNumber == 1){
        if (visualise){
            worldModel->getRVSender()->drawText("BallPos",std::to_string(ball.getX())+","+std::to_string(ball.getY()),ball.getX(),ball.getY()+1,0,0,0);
            worldModel->getRVSender()->drawText("BallDistance",std::to_string(GoalieDistanceToBall),pos.getX(),pos.getY()+1,0,0,0);
        }
    }


    
    if(GoalieDistanceToBall < 0.09){
        useGoalieCatch = true;
        // cout << "catch" << endl;
    }
    else{
        useGoalieCatch = false;

    }


}

void NaoBehavior::DetermineUsePass(vector<pair<double,int > > OppDistToBall){
    
    if(isShootingRange(ball, SHOOTING_RANGE)){
        usePass = false;
    }
    else if(OppDistToBall[0].first < 1){
        usePass = false;
    }
    else if(OppDistToBall[0].first < 2){
        usePass = true;
    }
}

VecPosition NaoBehavior::OptimalPassPosition(int _playerNumber, VecPosition initialtarget, vector<int> PointPreferences, vector<VecPosition> ImportantPositions){
    vector<pair<double,int > > TeamDistToInitialtarget = GenerateTeamToTargetDistanceVector(_playerNumber, initialtarget);

     bool found = false;
        int currIndex = 0;
        while(!found){
            int pNum = TeamDistToInitialtarget[currIndex].second;
            if(_playerNumber != pNum){

                VecPosition tempTarget = worldModel->getWorldObject(pNum)->pos;

                int mydesiredpositionIndex = PointPreferences[pNum-1]; 
                VecPosition mydesiredposition = ImportantPositions[mydesiredpositionIndex];
                VecPosition playerMovementVector = (mydesiredposition - tempTarget).normalize() *  (sqrt((ball - tempTarget).getMagnitude())/2);
                tempTarget = tempTarget + playerMovementVector + VecPosition(0.5,0,0); 

                if(isThereObstructionBetweenTargetAndBall(tempTarget)){

                }
                else{
                    if(false){
                        double distToTarget = ball.getDistanceTo(tempTarget);
                        if(distToTarget>2){ // prevent very close range pass
                            return tempTarget;
                        }             
                    }
                    else if(tempTarget.getX() > ball.getX()){ //prevent back pass
                        double distToTarget = ball.getDistanceTo(tempTarget);
                        if(distToTarget>2){ // prevent very close range pass
                            return tempTarget;
                        }
                    }

                    
                }
            }

            currIndex++;
            if(currIndex>5){ // if we cant find a viable pass from the 4 closest players then move on
                found = true;
            }
        }
    return initialtarget;
}


std::map<std::string, double> NaoBehavior::getCurrentStateDictionaryRL(bool updateTimeSteps){
    if (updateTimeSteps){
        ++mycount_2;
        mycount = (mycount + 1) % myRLAgent.observation_space.mod_timesteps; // hard code
        // mycount = (mycount + 1) % 30 ; // hard code
    }
    // printf("TIMESTEPS %d %d\n", mycount_2, mycount);
    std::map<std::string, double> m;
    // joint names
    for (int j=0; j < NUM_JOINTS; ++j){
        Joint jj = Joint(j);
        std::string this_joint_name = getJointName(jj);
        // m[getJointName(jj)] = raw_joint_angles_->values_[j];
        // this is in in radians, but we need degrees
        double updated = 180.0 * raw_joint_angles_->values_[j] / PI;
        // printf("Michael %s Joint %d has value of %lf and converted ratio of %lf", getJointName(jj), j, raw_joint_angles_->values_[j], updated);
        m[this_joint_name] = updated;
        
        // velocity calculations and suchlike

        if (current_state.find(this_joint_name) == current_state.end()){
            // this key is not in the current state -- so set velocity as zero
            m["VEL_" + this_joint_name] = 0.0;
        } else{
            m["VEL_" + this_joint_name] = (updated - current_state[this_joint_name]) / 0.02;
        }
    }
    // need to overwrite this
    std::string things = getJointName(Joint(RHipYawPitch));
    m[things] = oldValueRHipYawPitch * 180.0 / PI;
    
    m["VEL_" + things] = current_state.find(things) == current_state.end() ? 0.0 : (m[things] - currStateOldValueRHipYawPitch) / 0.02;


    // acc and gyro
    for (int j=0; j < NUM_SENSORS; ++j){
        Sensor jj = Sensor(j);
        m[getSensorString(jj)] = raw_sensors_->values_[j];
    }
    // ballpos
    double bx = ball.getX(), by = ball.getY(), bz = ball.getZ();

    // TODO, maybe make these default as ball pos should have no effect.
    m["ballposx"] = -3.0;  // bx;
    m["ballposy"] =  0.0;  // by;
    m["ballposz"] =  0.04; // bz;

    // ballpos relative
    auto me = worldModel->getMyPosition();
    double mx = me.getX(), my = me.getY(), mz = me.getZ();

    double rbx = (mx - bx), rby = (my - by), rbz = (mz - bz);

    double r = sqrt(rbx * rbx + rby * rby + rbz * rbz);
    double asin_input = rby / sqrt(rbx * rbx + rby * rby);
    double phi = abs(asin_input) <= 1 ? asin(asin_input)  : 0;
    double theta = acos(rbz/r);
    // std::cout << "Ballpos mx my mz" << mx << " " << my << " " << mz << " | bx by bz " << bx << " " << by << " " << bz << " | rbx rby rbz " << rbx << " " << rby << " " << rbz << " | r asininp phi theta " << r << " " << asin_input << " " << phi << " " << theta << "\n";

    // hardcode from Python: TODO
    // m["ballposrelx"] = 0.05316013544000806; //clip(r / 10.0, 0, 1);
    // m["ballposrely"] = 0.8732349425615492; //theta / PI;
    // m["ballposrelz"] = 0.42202086962263075; //(phi + PI / 2) / PI;
    m["ballposrelx"] = clip(r / 10.0, 0, 1);
    m["ballposrely"] = theta / PI;
    m["ballposrelz"] = (phi + PI / 2) / PI;
    
    // TODO calculate analytically.
    m["ballposrelx"] = 1.000000;// = 0.53452;
    m["ballposrely"] = 0.491944;// = 0.47765;
    m["ballposrelz"] = 0.499608;// = 0.33456;


    m["leftFootResistance_x"] =  bodyModel->getFRPCentreLeft()  .getX();
    m["leftFootResistance_y"] =  bodyModel->getFRPCentreLeft()  .getY();
    m["leftFootResistance_z"] =  bodyModel->getFRPCentreLeft()  .getZ();
    m["leftFootResistance_u"] =  bodyModel->getFRPForceLeft()   .getX();
    m["leftFootResistance_v"] =  bodyModel->getFRPForceLeft()   .getY();
    m["leftFootResistance_w"] =  bodyModel->getFRPForceLeft()   .getZ();
    
    m["rightFootResistance_x"] = bodyModel->getFRPCentreRight() .getX();
    m["rightFootResistance_y"] = bodyModel->getFRPCentreRight() .getY();
    m["rightFootResistance_z"] = bodyModel->getFRPCentreRight() .getZ();
    m["rightFootResistance_u"] = bodyModel->getFRPForceRight()  .getX();
    m["rightFootResistance_v"] = bodyModel->getFRPForceRight()  .getY();
    m["rightFootResistance_w"] = bodyModel->getFRPForceRight()  .getZ();
    
    m["timesteps"] = mycount;
    
    current_state = m;
    currStateOldValueRHipYawPitch = oldValueRHipYawPitch * 180.0 / PI;
    return m;
}


void NaoBehavior::warmup_rl_models(){
    if(fFatProxy)return;
    for (int i=0; i < all_rl_agents_indexed_by_int.size(); ++i){
        // break;
        // printf("Michael here\n"); fflush(stdout);
        for (int j=0; j < 5; ++j){
            // auto beg = high_resolution_clock::now();
            auto inputs = torch::ones(all_rl_agents_indexed_by_int[i].observation_space.observation_names.size() * all_rl_agents_indexed_by_int[i].observation_space.frame_stacking).to_dense();//.to(torch::kFloat64).to(torch::kFloat32);
            std::vector<torch::jit::IValue> inputs2;
            inputs2.push_back(inputs);
            auto output = all_rl_agents_indexed_by_int[i].rl_model.forward(inputs2).toTuple();

            // auto inputs2 = torch::zeros(all_rl_agents_indexed_by_int[i].observation_space.observation_names.size() * all_rl_agents_indexed_by_int[i].observation_space.frame_stacking);
            // auto output2 = all_rl_agents_indexed_by_int[i].rl_model.forward(inputs2).toTuple();
            // auto end = high_resolution_clock::now();
            // auto duration = duration_cast<microseconds>(end - beg);
            // std::cout << "\t\tStartup: " << i <<", " << j << duration.count() <<" microseconds\n";
        }
    }
}


RLAgent NaoBehavior::setUpRLAgent(){
    auto beg1 = high_resolution_clock::now();
    // std::string ENV_EXPORT_NAME = "/home/robocup/robocup/RoboCup/optimization/low-level-optim/wits_gym/wits_gym/export.env";
    // std::string RL_CHECKPOINT_NAME = "/home/robocup/robocup/RoboCup/optimization/low-level-optim/wits_gym/wits_gym/export/v1/TORCH_CPP_MODEL.pt";
    // std::string ENV_EXPORT_NAME = "./optimization/low-level-optim/wits_gym/wits_gym/export.env";
    #ifdef IS_RL_TRAIN
        std::string ENV_EXPORT_NAME = "./optimization/low-level-optim/wits_gym/wits_gym/env_specs/export.env." + std::to_string(RLPort);
    #else
        // std::string ENV_EXPORT_NAME = "./optimization/low-level-optim/wits_gym/wits_gym/export.env";
        // std::string ENV_EXPORT_NAME = "./behaviors/rl/rl_models/kick/fast_short_344/export.env";
        // std::string ENV_EXPORT_NAME = "./behaviors/rl/rl_models/kick/slow_far/export.env";
        std::string ENV_EXPORT_NAME = "./behaviors/rl/rl_models/kick/fast_short/export.env";
    #endif
    std::cout << "USING EXPORT ENV FILE OF: " << ENV_EXPORT_NAME << std::endl;
    // std::string RL_CHECKPOINT_NAME = "./optimization/low-level-optim/wits_gym/wits_gym/export/v1/TORCH_CPP_MODEL.pt";
    // std::string RL_CHECKPOINT_NAME = "./behaviors/rl/rl_models/kick/fast_short_344/TORCH_CPP_MODEL.pt";
    std::string RL_CHECKPOINT_NAME = "./behaviors/rl/rl_models/kick/fast_short/TORCH_CPP_MODEL.pt";
    // std::string RL_CHECKPOINT_NAME = "./behaviors/rl/rl_models/kick/slow_far/TORCH_CPP_MODEL.pt";

    int IDX = 0;
    auto get_thing = [](std::string s){
        return RLAgent::makeRLAgent("./behaviors/rl/rl_models/kick/" + s + "/export.env", "./behaviors/rl/rl_models/kick/" + s + "/TORCH_CPP_MODEL.pt");
    };

    auto add_thing = [&](std::string s, std::string name){
        all_rl_agents_indexed_by_int.push_back(get_thing(s));
        all_rl_agents_index_map[name] = IDX;
        IDX++;
    };

    // specific names
    // std::vector<std::string> ALL_RL_MODELS = {"fast_short", "fast_short_344", "fast_short_398", "slow_far", "slow_long_367", "slow_long_382_agent_type_1", "slow_long_383_agent_type_2", "slow_long_384_agent_type_3"};
    // for (auto s: ALL_RL_MODELS){
        // all_my_rl_agents.emplace(s, get_thing(s)); // RLAgent::makeRLAgent("./behaviors/rl/rl_models/kick/" + s + "/export.env", "./behaviors/rl/rl_models/kick/" + s + "/TORCH_CPP_MODEL.pt");
            // add_thing(s, s);
    // }
    // add_thing("fast_short_398", "normal"); // pretty good, angles not as stable


    add_thing("fast_short", "normal"); // quite stable, slightly shorter than the other one
    // add_thing("fast_short_398_longer_train", "normal"); //
    
    
    // if (agentBodyType == 0)
    //     add_thing("slow_far", "long");
    // else if (agentBodyType == 1)
    //     add_thing("slow_long_382_agent_type_1", "long");
    // else if (agentBodyType == 2)
    //     add_thing("slow_long_383_agent_type_2", "long");
    // else if (agentBodyType == 3)
    //     add_thing("slow_long_384_agent_type_3", "long");
    // else
    if (isAPenaltyKicker()){
        if (agentBodyType == 1)      add_thing("slow_long_382_agent_type_1", "long");
        else if (agentBodyType == 2) add_thing("slow_long_383_agent_type_2", "long");
        else if (agentBodyType == 3) add_thing("slow_long_384_agent_type_3", "long");
        else add_thing("slow_far", "long");

    }else{
        if (agentBodyType == 3)
            add_thing("slow_long_384_agent_type_3", "long");
        else
            add_thing("slow_far", "long");
    }

    // add_thing("fast_no_pid", "fast_no_pid");
    // add_thing("fast_no_pid_longer_train", "fast_no_pid");
    
    // if (agentBodyType == 3) add_thing("fast_no_pid_agent_type_3", "fast_no_pid");
    if (agentBodyType == 3) add_thing("fast_no_pid_longer_train", "fast_no_pid");
    else if (agentBodyType == 2) add_thing("fast_no_pid_agent_type_2", "fast_no_pid");
    else if (agentBodyType == 1) add_thing("fast_no_pid_agent_type_1", "fast_no_pid");
    else add_thing("fast_no_pid_longer_train", "fast_no_pid");

    // all_rl_agents_indexed_by_int.push_back(get_thing(s));
    // all_rl_agents_index_map["normal"] = IDX;
    // IDX++;
    // MY_RL_AGENT_NORMAL = get_thing("fast_short_398");
    // MY_RL_AGENT_LONG   = get_thing("slow_long_367");

    // understandable names
    // all_my_rl_agents["normal"] = get_thing("fast_short_398") ;//all_my_rl_agents["fast_short_398"];
    // all_my_rl_agents["long"]   = get_thing("slow_long_367") ;//all_my_rl_agents["slow_long_367"];
    // all_my_rl_agents.emplace("normal", std::make_unique<RLAgent>(get_thing("fast_short_398")));
    // all_my_rl_agents.emplace("long",   get_thing("slow_long_367"));
    // return get_thing("fast_short_398");
    
    auto end1 = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(end1 - beg1);
    std::cout << "\tLOADING: " << duration.count() <<" microseconds\n";
    return get_rl_agent_from_name("fast_short");
    return RLAgent::makeRLAgent(ENV_EXPORT_NAME, RL_CHECKPOINT_NAME);

    // std::cout << ("Hi there, setting up RL agent\n");
    // std::vector<std::string> effector_names, obs_names;
    // std::vector<double> min_obs, max_obs;
    // RLAction _action_space (effector_names, PID_DESIRED_MAX_SPEED);
    // RLObservation _observation_space(obs_names, min_obs, max_obs);
    // torch::jit::script::Module _rl_model = torch::jit::load(RL_CHECKPOINT_NAMES);
    // RLAgent agent(_action_space, _observation_space, _rl_model);

    // std::vector<torch::jit::IValue> inputs;
    // inputs.push_back(torch::ones({47}));
    // auto output = *_rl_model.forward(inputs).toTuple();
    // auto v2 = output.elements()[0].toTensor().slice(0, 0, 1);
    // std::cout << "RL Action test " << v2.item() << '\n';

    // return agent;
}