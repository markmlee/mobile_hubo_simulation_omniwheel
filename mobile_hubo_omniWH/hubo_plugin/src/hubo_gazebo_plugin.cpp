
#include "hubo_gazebo_plugin.h"

#define PODO_ADDR       "10.12.3.30"
#define PODO_PORT       8888

const float     D2Rf = 0.0174533;
const float     R2Df = 57.2957802;
char ip[20];

int jptrs_i = 0;

namespace gazebo
{

void DRCPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    std::cout << "\033[1;32m===================================" << std::endl;
    std::cout << "   Now Start the DRCPlugin..!!" << std::endl << std::endl;
    std::cout << "   Developer: Jeongsoo Lim" << std::endl;
    std::cout << "   E-mail   : yjs0497@kaist.ac.kr" << std::endl;
    std::cout << "===================================\033[0m" << std::endl;

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized()){
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }

    filt_alpha = 0.05;
    new_ref_cnt = 0;
    home_ref_RWH = home_ref_LWH = 0.0;

    model = _model;
    model->SetGravityMode(false);
    JCon = new physics::JointController(model);

    std::cout << "Model Name : " << model->GetName() << std::endl;

    // Create Socket ---------------------
    FILE *fpNet = NULL;
    fpNet = fopen("/home/rainbow/catkin_ws/src/mobile_hubo_omniWH/settings/network.txt", "r");
    if(fpNet == NULL){
        std::cout << ">>> Network File Open Error..!!" << std::endl;
        sprintf(ip, PODO_ADDR);
    }else{
        std::cout << ">>> Network File Open Success..!!" << std::endl;
        fscanf(fpNet, "%s", ip);
        fclose(fpNet);
    }

    if(CreateSocket(ip, PODO_PORT)){
        std::cout << "Created Socket.." << std::endl;
        RXDataSize = sizeof(DRC_GAZEBO_JOINT);
        TXDataSize = sizeof(DRC_GAZEBO_SENSOR);
        RXBuffer = (void*)malloc(RXDataSize);
        TXBuffer = (void*)malloc(TXDataSize);
        int threadID = pthread_create(&LANTHREAD_t, NULL, &LANThread, this);
        if(threadID < 0){
            std::cout << "Create Thread Error.." << std::endl;
        }
    }else{
        std::cout << "Create Socket Error.." << std::endl;
    }



    // Load Gains for Joints --------------
    FILE *fpGain = NULL;
    fpGain = fopen("/home/rainbow/catkin_ws/src/mobile_hubo_omniWH/settings/gain.txt", "r");
    if(fpGain == NULL){
        std::cout << ">>> Gain File Open Error..!!" << std::endl;
    }else{
        std::cout << ">>> Gain File Open Success..!!" << std::endl;
        for(int i=0; i<NO_OF_JOINTS; i++){
            fscanf(fpGain, "%f, %f, %f\n", &PIDGains[i][0], &PIDGains[i][1], &PIDGains[i][2]);
        }
        fclose(fpGain);
    }




    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND){
            jptrs_i++;
            continue;
        }
        else if(i < 12 || i > 28)
            continue;

        JPtrs[jptrs_i] = model->GetJoint(JointNameList[i]);
        std::cout <<"i = " << i <<", Joint Name: "<< JointNameList[i] << ", JPtrs["<< jptrs_i <<"] = " << model->GetJoint(JointNameList[i]) << std::endl;
        JCon->AddJoint(JPtrs[jptrs_i]);
        JCon->SetPositionPID(JPtrs[jptrs_i]->GetScopedName(),common::PID(PIDGains[i][0],PIDGains[i][1],PIDGains[i][2]));
        setGainOverride(jptrs_i, 100, 5);
        jptrs_i++;
    }


    for(int i=0; i<5; i++)
        adjustAllGain();

    JPtr_LHAND[0] = model->GetJoint("LHAND_a1");
    JPtr_LHAND[1] = model->GetJoint("LHAND_a2");
    JPtr_LHAND[2] = model->GetJoint("LHAND_a3");
    JPtr_LHAND[3] = model->GetJoint("LHAND_b1");
    JPtr_LHAND[4] = model->GetJoint("LHAND_b2");
    JPtr_LHAND[5] = model->GetJoint("LHAND_b3");
    JPtr_LHAND[6] = model->GetJoint("LHAND_c1");
    JPtr_LHAND[7] = model->GetJoint("LHAND_c2");
    JPtr_LHAND[8] = model->GetJoint("LHAND_c3");

    JPtr_RHAND[0] = model->GetJoint("RHAND_a1");
    JPtr_RHAND[1] = model->GetJoint("RHAND_a2");
    JPtr_RHAND[2] = model->GetJoint("RHAND_a3");
    JPtr_RHAND[3] = model->GetJoint("RHAND_b1");
    JPtr_RHAND[4] = model->GetJoint("RHAND_b2");
    JPtr_RHAND[5] = model->GetJoint("RHAND_b3");
    JPtr_RHAND[6] = model->GetJoint("RHAND_c1");
    JPtr_RHAND[7] = model->GetJoint("RHAND_c2");
    JPtr_RHAND[8] = model->GetJoint("RHAND_c3");

    for(int i=0; i<9; i++){
        JCon->AddJoint(JPtr_RHAND[i]);
        JCon->SetPositionPID(JPtr_RHAND[i]->GetScopedName(),common::PID(20,0,0.01));
        JCon->AddJoint(JPtr_LHAND[i]);
        JCon->SetPositionPID(JPtr_LHAND[i]->GetScopedName(),common::PID(20,0,0.01));
    }

    JPtr_RWFT = model->GetJoint("RWFT");
    JPtr_LWFT = model->GetJoint("LWFT");

    JCon->AddJoint(JPtr_RWFT);
    JCon->AddJoint(JPtr_LWFT);

    JCon->SetPositionPID(JPtr_RWFT->GetScopedName(),common::PID(0,0,0.001));
    JCon->SetPositionPID(JPtr_LWFT->GetScopedName(),common::PID(0,0,0.001));

    jptrs_i = 0;
    // provide feedback for getting wrench---
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND){
            jptrs_i++;
            continue;
        }
        else if(i<12 || i>28)
            continue;
        JPtrs[jptrs_i]->SetProvideFeedback(true);
        jptrs_i++;
    }

    JPtr_RWFT->SetProvideFeedback(true);
    JPtr_LWFT->SetProvideFeedback(true);
    // ---------------------------------------

    for(int i=0;i<=mHUBO_NO_OF_JOINTS;i++){
        refs[i] = 0.0;
    }
    ref_RHAND = ref_LHAND = 1.4;//1.4708;

    refs[LEB-12] += -20*D2Rf;
    refs[REB-12] += -20*D2Rf;
    refs[RSR-12] += -15*D2Rf;
    refs[LSR-12] += 15*D2Rf;


    //initialize WHeel variable
    for(int i=0; i<3; i++){
        pos_base[i] = 0;
    }

    jptrs_i =0;
    for(int i=0; i<=NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND){
            jptrs_i++;
            continue;
        }
        else if(i<12 || i>28)
            continue;

        JCon->SetJointPosition(JPtrs[jptrs_i],refs[jptrs_i]);
        jptrs_i++;
    }
    JCon->SetJointPosition(JPtr_RWFT,0);
    JCon->SetJointPosition(JPtr_LWFT,0);

    if(!LAFT || !RAFT || !IMU)
        std::cout << "SENSOR ERROR (NULL)" << std::endl;

    // Listen to the update event. This event is broadcast every simulation iteration.
    world = model->GetWorld();
    UpdateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DRCPlugin::OnUpdate, this, _1));
}

void DRCPlugin::OnUpdate(const common::UpdateInfo &){
    static int cnt = 0;

    if(FirstRcvData){
        if(FirstRcvCnt == 0){
            std::cout << "FirstRcvCnt == 0 " << std::endl;
            model->SetGravityMode(false);
            model->SetWorldPose(math::Pose(0,0,2, 0,0,0));
            for(int i=0; i<mHUBO_NO_OF_JOINTS; i++){
                setGainOverride(i, 100, 20);
            }
        }else if(FirstRcvCnt == 21){
            std::cout << "FirstRcvCnt == 21 " << std::endl;
            jptrs_i=0;
            for(int i=0; i<NO_OF_JOINTS; i++){
                if(i == RHAND || i == LHAND ){
                    jptrs_i++;
                    continue;
                }
                else if(i<12 || i>28)
                    continue;
                refs[jptrs_i] = RXJointData.JointReference[i]*D2Rf;
                //****************DEBUGGING*******************//
                std::cout << "refs["<<jptrs_i<<"] = "<< refs[jptrs_i] <<", RXJointData.JointReference[" << i << "] = " << RXJointData.JointReference[i]*D2Rf << std::endl;
                jptrs_i++;
            }

            //        ref_RHAND   += RXJointData.JointReference[RHAND]*1.4708/40/2000;
            //        ref_LHAND   += RXJointData.JointReference[LHAND]*1.4708/40/2000;
            //        if(ref_RHAND>1.4708)    ref_RHAND = 1.4708;
            //        if(ref_RHAND<0)         ref_RHAND = 0;
            //        if(ref_LHAND>1.4708)    ref_LHAND = 1.4708;
            //        if(ref_LHAND<0)         ref_LHAND = 0;
            ref_RHAND   += RXJointData.JointReference[RHAND]*1.4/40/2000;
            ref_LHAND   += RXJointData.JointReference[LHAND]*1.4/40/2000;
            if(ref_RHAND>1.4)    ref_RHAND = 1.4;
            if(ref_RHAND<0)      ref_RHAND = 0;
            if(ref_LHAND>1.4)    ref_LHAND = 1.4;
            if(ref_LHAND<0)      ref_LHAND = 0;

            //****************DEBUGGING*******************//
            std::cout << "RXJointData.JointReference[LHAND] = " << RXJointData.JointReference[LHAND] << std::endl;
            std::cout << "RXJointData.JointReference[RHAND] = " << RXJointData.JointReference[RHAND] << std::endl;

            // default offset
            refs[LEB-12] += -20*D2Rf;
            refs[REB-12] += -20*D2Rf;
            refs[RSR-12] += -15*D2Rf;
            refs[LSR-12] +=  15*D2Rf;

            for(int i=0; i<mHUBO_NO_OF_JOINTS; i++){
                setGainOverride(i, 0, 1000);
            }
        }else if(FirstRcvCnt == 3200){
            std::cout << "FirstRcnCnt == 3200" << std::endl;
            float z;
            try{
                z =0.123;
            }catch(tf::TransformException &ex){
                ROS_WARN("%s",ex.what());
                z = 0.122;
            }
            model->SetWorldPose(math::Pose(0,0,z, 0,0,0));
            model->GetWorld()->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,-2.9));
            model->SetGravityMode(true);
        }else if(FirstRcvCnt == 6500){
            std::cout << "FirstRcnCnt == 6500" << std::endl;
            model->GetWorld()->GetPhysicsEngine()->SetGravity(math::Vector3(0,0,-9.8));
            FirstRcvData = false;
            SimulateOk = true;
        }
    }


    if(SimulateOk){

        jptrs_i = 0;
        for(int i=0; i<NO_OF_JOINTS; i++){
            if(i == RHAND || i == LHAND ){
                jptrs_i++;
                continue;
            }
            else if(i<12 || i>28)
                continue;

            refs[jptrs_i] = RXJointData.JointReference[i]*D2Rf;
            //std::cout<< "refs["<<jptrs_i<<"] = " << refs[jptrs_i] << std::endl;
            jptrs_i++;
        }
//        ref_RHAND   += RXJointData.JointReference[RHAND]*1.4708/40/2000;
//        ref_LHAND   += RXJointData.JointReference[LHAND]*1.4708/40/2000;
//        if(ref_RHAND>1.4708)    ref_RHAND = 1.4708;
//        if(ref_RHAND<0)         ref_RHAND = 0;
//        if(ref_LHAND>1.4708)    ref_LHAND = 1.4708;
//        if(ref_LHAND<0)         ref_LHAND = 0;
        ref_RHAND   += RXJointData.JointReference[RHAND]*1.4/40/2000;
        ref_LHAND   += RXJointData.JointReference[LHAND]*1.4/40/2000;
        if(ref_RHAND>1.4)    ref_RHAND = 1.4;
        if(ref_RHAND<0)      ref_RHAND = 0;
        if(ref_LHAND>1.4)    ref_LHAND = 1.4;
        if(ref_LHAND<0)      ref_LHAND = 0;

        // default offset
        refs[LEB-12] += -20*D2Rf;
        refs[REB-12] += -20*D2Rf;
        refs[RSR-12] += -15*D2Rf;
        refs[LSR-12] += 15*D2Rf;

        //Base movement
        ref_BASE[0] = RXJointData.JointReference[RWH];
        ref_BASE[1] = RXJointData.JointReference[LWH];
        ref_BASE[2] = RXJointData.JointReference[RAP];

        for (int i =0; i < 3; i++)
            inc_ref_BASE[i] = ref_BASE[i] - prev_ref_BASE[i];

        if(inc_ref_BASE[0] ==0 && inc_ref_BASE[1] == 0 && inc_ref_BASE[2] == 0){
        }else{
            std::cout << "move Base" << std::endl;
            MoveBase();
        }

    }


    // move joints
    jptrs_i = 0;
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND ){
            jptrs_i++;
            continue;
        }
        else if( i<12 || i>28 )
            continue;

        JCon->SetPositionTarget(JPtrs[jptrs_i]->GetScopedName(),refs[jptrs_i]);

        //***DEBUGGING***
//        if( i==13 || i ==19 || i ==16 || i == 22){
//            std::cout<<"RXdata["<<i<<"] = " << RXJointData.JointReference[i]<< std::endl;
//            std::cout<<"refs["<<jptrs_i<<"]= " << refs[jptrs_i]<< std::endl;
//        }

        jptrs_i++;
    }
//    std::cout << std::endl;

    for(int i=0; i<9; i++){
        JCon->SetPositionTarget(JPtr_RHAND[i]->GetScopedName(), ref_RHAND);
        JCon->SetPositionTarget(JPtr_LHAND[i]->GetScopedName(), ref_LHAND);
    }

    JCon->Update();

    // adjust gain override
    adjustAllGain();

    // RWFT, LWFT ------
    physics::JointWrench wrenchR = JPtr_RWFT->GetForceTorque(0);
    math::Vector3 RWF = wrenchR.body1Force;
    math::Vector3 RWT = wrenchR.body1Torque;
    filt_RWF = filt_alpha*RWF + (1-filt_alpha)*filt_RWF;
    filt_RWT = filt_alpha*RWT + (1-filt_alpha)*filt_RWT;
    if(connectionStatus){
        TXSensorData.FTSensor[2].force[0]   = filt_RWF.x;
        TXSensorData.FTSensor[2].force[1]   = filt_RWF.y;
        TXSensorData.FTSensor[2].force[2]   = filt_RWF.z;
        TXSensorData.FTSensor[2].torque[0]  = filt_RWT.x;
        TXSensorData.FTSensor[2].torque[1]  = filt_RWT.y;
        TXSensorData.FTSensor[2].torque[2]  = filt_RWT.z;
    }
    physics::JointWrench wrenchL = JPtr_LWFT->GetForceTorque(0);
    math::Vector3 LWF = wrenchL.body1Force;
    math::Vector3 LWT = wrenchL.body1Torque;
    filt_LWF = filt_alpha*LWF + (1-filt_alpha)*filt_LWF;
    filt_LWT = filt_alpha*LWT + (1-filt_alpha)*filt_LWT;
    if(connectionStatus){
        TXSensorData.FTSensor[3].force[0]   = filt_LWF.x;
        TXSensorData.FTSensor[3].force[1]   = filt_LWF.y;
        TXSensorData.FTSensor[3].force[2]   = filt_LWF.z;
        TXSensorData.FTSensor[3].torque[0]  = filt_LWT.x;
        TXSensorData.FTSensor[3].torque[1]  = filt_LWT.y;
        TXSensorData.FTSensor[3].torque[2]  = filt_LWT.z;
    }

    //============================================================

    if(cnt%5==0){
        jptrs_i =0;
        if(connectionStatus){
            //std::cout << "SEND DATA??" << std::endl;

            // send sensor data
            for(int i=0; i<NO_OF_JOINTS; i++){
                if(i == RHAND || i == LHAND){
                    jptrs_i++;
                    continue;
                }
                else if(i<12 || i>28){
                    TXSensorData.JointCurrentPosition[i] = 0.0;
                    continue;
                }
                TXSensorData.JointCurrentPosition[i] = JPtrs[jptrs_i]->GetAngle(0).Radian()*R2Df;
                jptrs_i++;
                //std::cout << "TXSensorData.JointCurrentPosition["<<i<<"] = " << JPtrs[i]->GetAngle(0).Radian()*R2Df << std::endl;
            }
            TXSensorData.JointCurrentPosition[RHAND] = (JPtr_RHAND[0]->GetAngle(0).Radian())*R2Df;
            TXSensorData.JointCurrentPosition[LHAND] = (JPtr_LHAND[0]->GetAngle(0).Radian())*R2Df;

            // default offset
            TXSensorData.JointCurrentPosition[LEB] -= -20.0;
            TXSensorData.JointCurrentPosition[REB] -= -20.0;
            TXSensorData.JointCurrentPosition[RSR] -= -15.0;
            TXSensorData.JointCurrentPosition[LSR] -=  15.0;

            common::Time simTime = model->GetWorld()->GetSimTime();
            ros::Time rosTime = ros::Time::now();

            TXSensorData.Sim_Time.sec = simTime.sec;
            TXSensorData.Sim_Time.nsec = simTime.nsec;
            TXSensorData.ROS_Time.sec = rosTime.sec;
            TXSensorData.ROS_Time.nsec = rosTime.nsec;
            write(sock, &TXSensorData, sizeof(TXSensorData));
        }
    }
    cnt++;
    FirstRcvCnt++;
}



void DRCPlugin::OnLAFTUpdate(){
    math::Vector3 LAF = LAFT->Force();
    math::Vector3 LAT = LAFT->Torque();
    filt_LAF = filt_alpha*LAF + (1-filt_alpha)*filt_LAF;
    filt_LAT = filt_alpha*LAT + (1-filt_alpha)*filt_LAT;
    if(connectionStatus){
        TXSensorData.FTSensor[1].force[0]   = filt_LAF.x;
        TXSensorData.FTSensor[1].force[1]   = filt_LAF.y;
        TXSensorData.FTSensor[1].force[2]   = filt_LAF.z;
        TXSensorData.FTSensor[1].torque[0]  = filt_LAT.x;
        TXSensorData.FTSensor[1].torque[1]  = filt_LAT.y;
        TXSensorData.FTSensor[1].torque[2]  = filt_LAT.z;
    }
}

void DRCPlugin::OnRAFTUpdate(){
    math::Vector3 RAF = RAFT->Force();
    math::Vector3 RAT = RAFT->Torque();
    filt_RAF = filt_alpha*RAF + (1-filt_alpha)*filt_RAF;
    filt_RAT = filt_alpha*RAT + (1-filt_alpha)*filt_RAT;
    if(connectionStatus){
        TXSensorData.FTSensor[0].force[0]   = filt_RAF.x;
        TXSensorData.FTSensor[0].force[1]   = filt_RAF.y;
        TXSensorData.FTSensor[0].force[2]   = filt_RAF.z;
        TXSensorData.FTSensor[0].torque[0]  = filt_RAT.x;
        TXSensorData.FTSensor[0].torque[1]  = filt_RAT.y;
        TXSensorData.FTSensor[0].torque[2]  = filt_RAT.z;
    }
}

void DRCPlugin::OnIMUUpdate(){
//    math::Vector3 AV = IMU->AngularVelocity();
//    math::Vector3 LA = IMU->LinearAcceleration();
//    math::Quaternion q = IMU->Orientation();
//    if(connectionStatus){
//        float l = q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z;
//        float w, x, y, z;
//        if(l > 0.00001){
//            l = sqrt(l);
//            w = q.w/l; x = q.x/l; y = q.y/l; z = q.z/l;
//        }else{
//            w = 1; x = 0; y = 0; z = 0;
//        }
//        // roll, pitch, yaw
//        TXSensorData.IMUSensor[0] = atan2(2*(w*x + y*z), 1-2*(x*x + y*y))*R2Df;
//        TXSensorData.IMUSensor[1] = asin(2*(w*y - z*x))*R2Df;
//        TXSensorData.IMUSensor[2] = atan2(2*(w*z + x*y), 1-2*(y*y + z*z))*R2Df;
//        // rvel, pvel, yvel
//        TXSensorData.IMUSensor[3] = AV.x*R2Df;
//        TXSensorData.IMUSensor[4] = AV.y*R2Df;
//        TXSensorData.IMUSensor[5] = AV.z*R2Df;
//        // accx, accy, accz
//        TXSensorData.IMUSensor[6] = LA.x;
//        TXSensorData.IMUSensor[7] = LA.y;
//        TXSensorData.IMUSensor[8] = LA.z;
//    }
}


int DRCPlugin::CreateSocket(const char *addr, int port){
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1){
        return false;
    }
    client.sin_addr.s_addr = inet_addr(addr);
    client.sin_family = AF_INET;
    client.sin_port = htons(port);

    int optval = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    return true;
}


int DRCPlugin::Connect2Server(){    
    if(connect(sock, (struct sockaddr*)&client, sizeof(client)) < 0){
        return false;
    }
    std::cout << "Client connect to server!! (DRC_PLUGIN)" << std::endl;
    return true;
}

void DRCPlugin::NewRXData(){
    if(FirstRcvData == false && SimulateOk == false){
        FirstRcvCnt = 0;
        FirstRcvData = true;
    }

    // save previous refs
    for(int i=0; i<mHUBO_NO_OF_JOINTS; i++){
        prev_refs[i] = refs[i];
    }

    for(int i=0; i<3; i++){
        prev_ref_BASE[i] = ref_BASE[i];
    }

    // refresh new refs
    memcpy(&(RXJointData), RXBuffer, RXDataSize);

    new_ref_cnt = 1;
}


void *DRCPlugin::LANThread(void *_arg){
    DRCPlugin *dp = (DRCPlugin*)_arg;

    dp->threadWorking = true;
    dp->FirstRcvData = false;
    dp->SimulateOk = false;

    unsigned int tcp_status = 0x00;
    int tcp_size = 0;
    int connectCnt = 0;

    int readDone = true;
    int dataType = 0;

    int loopCounter = 0;

    while(dp->threadWorking){
        usleep(100);
        if(tcp_status == 0x00){
            // If client was not connected
            if(dp->sock == 0){
                dp->CreateSocket(ip, PODO_PORT);
            }
            if(dp->Connect2Server()){
                tcp_status = 0x01;
                dp->connectionStatus = true;
                connectCnt = 0;
            }else{
                if(connectCnt%10 == 0)
                    std::cout << "Connect to Server Failed.." << std::endl;
                connectCnt++;
            }
            usleep(1000*1000);
        }
        if(tcp_status == 0x01){
            // If client was connected
            if(readDone == true){
                tcp_size = read(dp->sock, &dataType, sizeof(int));
                if(tcp_size == sizeof(int)){
                    readDone = false;
                }
            }
            else if(readDone == false){
                switch(dataType){
                std::cout << "dataType = " << dataType << std::endl;
                case GAZEBO_TYPE_JOINT:
                    tcp_size = read(dp->sock, dp->RXBuffer, sizeof(DRC_GAZEBO_JOINT));
                    if(tcp_size == sizeof(DRC_GAZEBO_JOINT)){
                        dp->NewRXData();
                        readDone = true;
                        dataType = GAZEBO_TYPE_NO;
                    }
                    break;
                case GAZEBO_TYPE_GAINOVERRIDE:
                    tcp_size = read(dp->sock, &dp->GainOverrideCMD, sizeof(DRC_GAZEBO_GO_CMD));
                    if(tcp_size == sizeof(DRC_GAZEBO_GO_CMD)){
                        dp->setGainOverride(dp->GainOverrideCMD.joint, dp->GainOverrideCMD.gain, dp->GainOverrideCMD.timeMs);
                        readDone = true;
                        dataType = GAZEBO_TYPE_NO;
                    }
                    break;
                case GAZEBO_TYPE_HOME:
                    int joint;
                    tcp_size = read(dp->sock, &dp->HomeJoint, sizeof(int));
                    if(tcp_size == sizeof(int)){
                        dp->findHome(dp->HomeJoint);
                        readDone = true;
                        dataType = GAZEBO_TYPE_NO;
                    }
                    break;
                default:
                    //std::cout << "DATA TYPE  :" << dataType << std::endl;
                    break;
                }
            }

            // disconnected..
            if(tcp_size == 0){
                tcp_status = 0x00;
                dp->connectionStatus = false;
                dp->FirstRcvData = false;
                dp->SimulateOk = false;
                close(dp->sock);
                dp->sock = 0;
                std::cout << "Socket Disconnected.." << std::endl;
            }
        }
    }
    return NULL;
}

void DRCPlugin::setGainOverride(int joint, float gain, long msTime){
    GainOverride[joint].flag = false;
    GainOverride[joint].togo = gain;
    GainOverride[joint].init = GainOverride[joint].current;
    GainOverride[joint].delta = GainOverride[joint].togo - GainOverride[joint].current;
    GainOverride[joint].curCnt = 0;
    GainOverride[joint].goalCnt = msTime;
    GainOverride[joint].flag = true;
}

void DRCPlugin::moveGainOverride(int joint){
    if(GainOverride[joint].flag == true){
        GainOverride[joint].curCnt++;
        if(GainOverride[joint].goalCnt <= GainOverride[joint].curCnt){
            GainOverride[joint].goalCnt = GainOverride[joint].curCnt = 0;
            GainOverride[joint].current = GainOverride[joint].togo;
            GainOverride[joint].flag = false;
        }else{
            GainOverride[joint].current = GainOverride[joint].init + GainOverride[joint].delta*GainOverride[joint].curCnt/GainOverride[joint].goalCnt;
        }
    }
}

void DRCPlugin::adjustAllGain(){
    jptrs_i=0;
    for(int i=0; i<NO_OF_JOINTS; i++){
        if(i == RHAND || i == LHAND){
            jptrs_i++;
            continue;
        }
        else if(i<12 || i>28)
            continue;

        if(GainOverride[jptrs_i].flag == true){
            moveGainOverride(jptrs_i);
            double gain = pow(0.93, GainOverride[jptrs_i].current);
            JCon->SetPositionPID(JPtrs[jptrs_i]->GetScopedName(),common::PID(gain*PIDGains[i][0],gain*PIDGains[i][1],gain*PIDGains[i][2]));
        }
        jptrs_i++;
    }
}

void DRCPlugin::findHome(int jNum){
    // Only for RWH & LWH
    //if(jNum == RWH){
        //home_ref_RWH += ref_RWH;
        //ref_RWH = 0.0;
        //setGainOverride(RWH, 0, 10);
    //}else if(jNum == LWH){
        //home_ref_LWH += ref_LWH;
        //ref_LWH = 0.0;
        //setGainOverride(LWH, 0, 10);
    //}
}


void DRCPlugin::MoveModelsPlane(float position_x, float position_y, float position_z, float angular_x, float angular_y, float angular_z)
{
    model->GetJoint("RHY")->SetPosition(0,position_x);
    model->GetJoint("RHR")->SetPosition(0,position_y);
    model->GetJoint("RHP")->SetPosition(0,angular_z);
}

void DRCPlugin::MoveBase()
{
    //double a = 60, b = 180, c = -60; in degree
    Tr(0,0) = sin(a*D2Rf); Tr(0,1)=cos(a*D2Rf); Tr(0,2)=RobotR;
    Tr(1,0) = sin(b*D2Rf); Tr(1,1)=cos(b*D2Rf); Tr(1,2)=RobotR;
    Tr(2,0) = sin(c*D2Rf); Tr(2,1)=cos(c*D2Rf); Tr(2,2)=RobotR;

    for(int i=0; i<3; i++){
        prev_WH[i] = WH[i];
        WH[i] = ref_BASE[i];
        WH_v[i] = ((WH[i] - prev_WH[i])*200) * D2Rf * WheelR;
    }

    Vt <<WH_v[0] << arma::endr << WH_v[2] << arma::endr << WH_v[1] << arma::endr;
    Tr_inv = inv(Tr);
    Vr = Tr_inv * Vt;

    //rotation
    float ori[2], res[2]={0,0};
    ori[0] = Vr(0,0); ori[1] = Vr(1,0);
    Vr(0,0) = rotation_2D(ori, pos_base[2], 0);
    Vr(1,0) = rotation_2D(ori, pos_base[2], 1);

    for(int i=0; i<3; i++)
        pos_base[i] += Vr(i,0)*0.005;


    for(int i=0; i<3; i++)
        std::cout << "pos_base[" << i <<"] = " << pos_base[i] << std::endl;

    std::cout<<std::endl;


    MoveModelsPlane(pos_base[0], pos_base[1], 0.0, 0.0, 0.0, pos_base[2]);
}

float DRCPlugin::rotation_2D(float ori[2], float rot_angle,int col_num)
{
    arma::Mat<double> ori_mat     = arma::randu(2,1);
    arma::Mat<double> result_mat  = arma::randu(2,1);
    arma::Mat<double> rot_mat     = arma::randu(2,2);

    rot_mat << cos(rot_angle) << -sin(rot_angle) << arma::endr
            << sin(rot_angle) << cos(rot_angle)  << arma::endr ;

    ori_mat << ori[0]*1.0 << arma::endr
            << ori[1]*1.0 << arma::endr;

    result_mat = rot_mat * ori_mat;

    return result_mat(col_num,0);

}




GZ_REGISTER_MODEL_PLUGIN(DRCPlugin)

}
