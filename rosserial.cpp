#include "extern_globals.h"
#include <ros.h>
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include<geometry_msgs/Point.h>
#include <Dynamixel2Arduino.h>
#include "SACT_types.h"
#include "SACT_protocol.h"
// BOARD SUPPORT PACKAGE
#include "SACT_stdio.h"
// PROJECT SPECIFIC HEADERS
#include "extern_globals.h"
#include "SACT_Commands.h"
#include "cinematica.h"
#include <Arduino.h>
#include <sensor_msgs/JointState.h>
#include <stdlib.h>

ros::NodeHandle  nh;





void setup()
{

//msg_types

std_msgs::String errori;

sensor_msgs::JointState pos;

std_msgs::String data;

std_msgs::Int temps;

//inizializzo ros 
 

ros::Subscriber<std_msgs::Int> cmm("/control_mode_change",newCMM);
 
ros::Subscriber<sensor_msgs::JointState> SPR("/move/SPR",ExecSPR);

ros::Subscriber<geometry_msgs::Point>  SCR("/move/SCR",ExecSCR);

ros:Subscriber<std_msgs::String> demo_mode_req("/demo",DemoMode);


ros::Subscriber<std_msg::String> grip("/grip",GRIP_ON_OF);

ros::Subscriber<std_msgs::String> ssp("/info_request",SSP);

ros::Publisher pub_joint("/info/joint_state", &pos);

ros::Publisher pub_trq("/info/trq_data", &data);

ros::Publisher pub_temp("/info/temp_data", &temps);

ros::Publisher errors("/errors", errori);

nh.initNode();

nh.subscribe(cmm);

nh.advertise(errors);

nh.subscribe(ssp);

nh.advertise(pub_joint);

nh.advertise(pub_trq);

nh.advertise(pub_temp);

nh.subscribe(SPR);

nh.subscribe(SCR);

nh.subscribe(demo_mode);

nh.subscribe(grip);

nh.subscribe(nexec);

}


void newCMM(const std_msgs::Int& mode)
{
    if (control_mode.state == 0)) //OFF MODE
    {
        if (mode==1))
        {
            control_mode.state = POS_MODE;
        }

        else if (mode==2)
        {
            control_mode.state = CART_MODE;
        }

        else
           {
            errors.publish("ERROR IN CHANGING MODE");
            }

    }

    else if (control_mode.state==2) && mode==0) //SPR
    {
        
       {
       control_mode.state = OFF_MODE;
       }
       
       
    else

        {
        errors.publish("ONLY OFF MODE ALLOWED");
        }
    }
    else if (control_mode.state==1 && mode==0) //SPR
    {
        control_mode.state = OFF_MODE;
    }
    else

        {
        errors.publish("ONLY OFF MODE ALLOWED");
        }
    
	
}












void ExecSPR(const sensor_msgs::JointState& joint_pos) //joint
{     

                    if (control_mode.state==POS_MODE)
                    {
                       digitalWrite(20,LOW);
                      
                      //Converto angoli in RADIANTI per poter chiamare la cinematica diretta                      
                      angleJoints_temp.theta1=convert_decdeg_to_rad((float) joint_pos[0]);
                      angleJoints_temp.theta2=convert_decdeg_to_rad((float) joint_pos[1]);
                      angleJoints_temp.theta3=convert_decdeg_to_rad((float) joint_pos[2]);
                      
                      //Chiamo la cinematica diretta
                      if (delta_calcForward(&angleJoints_temp,&coordinates_temp)>=0)
                      {
                        //La cinematica è andata a buon fine
                        //Aggiungo l'OFFSET di scala ai decimi di grado
                        float temp_dyn1=convert_by_offset(joint_pos[0]);
                        float temp_dyn2=convert_by_offset(joint_pos[1]);
                        float temp_dyn3=convert_by_offset(joint_pos[2]);
                        
                   /*   //Converto gli angoli da DECIMI DI GRADO a UNITA' DYNAMIXEL da 0 a 1024, per muovere i motori
                        temp_dyn1=convert_decdeg_to_dyn(temp_dyn1);
                        temp_dyn2=convert_decdeg_to_dyn(temp_dyn2);
                        temp_dyn3=convert_decdeg_to_dyn(temp_dyn3);
                        */
                        
                        //Posso MUOVERE I MOTORI degli angoli calcolati theta1, theta2,theta3
                        Dxl.setGoalPosition(ID1,temp_dyn1,UNIT_DEGREE);
                        delayMicroseconds(ritardo);
                        Dxl.setGoalPosition(ID2,temp_dyn2,UNIT_DEGREE);
                        delayMicroseconds(ritardo);
                        Dxl.setGoalPosition(ID3,temp_dyn3,UNIT_DEGREE);
                        delayMicroseconds(ritardo);
                        
                                                
                            digitalWrite(20,HIGH);
                      }
                      else
                      {
                      	<std_msgs::String> err_msg ="SPR ERROR";
                        errors.publish(err_msg); //Stampa messaggio di errore
                      }
                     }
            }
                     
            	
            void ExecSCR(const geometry_msgs::Point& cartpos) //SCR
                    if (control_mode.state==CART_MODE)
                    {
                      //Converto in METRI le posizioni xyz inserite
                      coordinates_temp.x=convert_decmill_to_meters((float)cartpos.x);
                      coordinates_temp.y=convert_decmill_to_meters((float)cartpos.y);
                      coordinates_temp.z=convert_decmill_to_meters((float)cartpos.z);
                      
                      //Chiamo CINEMATICA INVERSA per risalire ai valori dei giunti
                      if (delta_calcInverse(&angleJoints_temp,&coordinates_temp)>=0)
                      {
                        //Converto in DECIMI DI GRADO i valori dei giunti calcolati
                        angleJoints_temp.theta1=convert_rad_to_decdeg(angleJoints_temp.theta1);
                        angleJoints_temp.theta2=convert_rad_to_decdeg(angleJoints_temp.theta2);
                        angleJoints_temp.theta3=convert_rad_to_decdeg(angleJoints_temp.theta3);
                      
                        //Aggiungo l'OFFSET di scala ai decimi di grado
                        angleJoints_temp.theta1=convert_by_offset(angleJoints_temp.theta1);
                        angleJoints_temp.theta2=convert_by_offset(angleJoints_temp.theta2);
                        angleJoints_temp.theta3=convert_by_offset(angleJoints_temp.theta3);
                        
                       /* //Converto in unità di misura DYNAMIXEL per poter muovere i motori
                        angleJoints_temp.theta1=convert_decdeg_to_dyn(angleJoints_temp.theta1);
                        angleJoints_temp.theta2=convert_decdeg_to_dyn(angleJoints_temp.theta2);
                        angleJoints_temp.theta3=convert_decdeg_to_dyn(angleJoints_temp.theta3);
                        */
                        
                        //Chiamo funzioni DXL per muovere i motori
                        Dxl.setGoalPosition(ID1,angleJoints_temp.theta1,UNIT_DEGREE);
                        Dxl.setGoalPosition(ID2,angleJoints_temp.theta2,UNIT_DEGREE);
                        Dxl.setGoalPosition(ID3,angleJoints_temp.theta3,UNIT_DEGREE);

                      }
                      else
                      {
                     
                        errors.publish("ERRORE SCR"); //Stampa messaggio di errore
                      }
                      }
                    
            
            


            void DemoMode(const std_msgs::String& demo) //la stringa deve essere del tipo "DEM 1"
                     {
                    	
                       //DEMO 1, SPOSTA UN OGGETTO DAL BASSO (-450 -450) ALL'ALTO (450 450) E TORNA INDIETRO. OK!!!!
                       if (strcmp(demo, "DEM 1"))
                       {
                            
                       for (k=0;k<1;k=k)
                       {
                             Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalPosition(ID1,convert_by_offset(495),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(100),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(407),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalPosition(ID1,convert_by_offset(697),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(340),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(621),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalVelocity(ID1,50);
                             Dxl.setGoalVelocity(ID2,50);
                             Dxl.setGoalVelocity(ID3,50);
                             Dxl.setGoalPosition(ID1,convert_by_offset(744),UNIT_DEGREE);//QUA SPESSO L'OGGETTO SI SPOSTA VERSO L'ALTO QUANDO LO DEVE PRENDERE, SPINGE TROPPO VERSO IL BASSO FORSE. E' PROPRIO IL DELTA CHE SI MUOVE IN GIU'...
                             Dxl.setGoalPosition(ID2,convert_by_offset(375),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(656),UNIT_DEGREE);
                             delay(1000);                             
                             Dxl.setGoalPosition(ID4,256);
                             delay(1000);
                             Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                             Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                             Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                             Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalPosition(ID1,convert_by_offset(161),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(551),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(281),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalPosition(ID1,convert_by_offset(401),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(744),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(513),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalPosition(ID4,500);
                             delay(1000);
                             Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalPosition(ID1,convert_by_offset(161),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(551),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(281),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalPosition(ID1,convert_by_offset(340),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(683),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(442),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalVelocity(ID1,50);
                             Dxl.setGoalVelocity(ID2,50);
                             Dxl.setGoalVelocity(ID3,50);
                             Dxl.setGoalPosition(ID1,convert_by_offset(425),UNIT_DEGREE);//434  //QUA SPESSO L'OGGETTO SI SPOSTA VERSO L'ALTO QUANDO LO DEVE PRENDERE, SPINGE TROPPO VERSO IL BASSO FORSE. 
                             Dxl.setGoalPosition(ID2,convert_by_offset(753),UNIT_DEGREE);//771
                             Dxl.setGoalPosition(ID3,convert_by_offset(522),UNIT_DEGREE);//530
                             delay(1000);
                             Dxl.setGoalPosition(ID4,256);
                             delay(1000);
                             Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                             Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                             Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                             Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalPosition(ID1,convert_by_offset(495),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(100),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(407),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalPosition(ID1,convert_by_offset(697),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(340),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(621),UNIT_DEGREE);
                             delay(1000);
                             Dxl.setGoalPosition(ID4,500);
                             delay(1000);
                             Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                             Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                             
                       }
                       }
                       else if(strcmp(demo,"DEM2"))  //SPOSTA DUE OGGETTI AFFIANCATI (-450 -450) E LI AFFIANCA NELL'ALTRO PALLET (450 450), POI TORNA INDIETRO
                       {
                       for (k=0;k<nesecuzioni;k)
                       {
                               Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(407),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(94),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(498),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(563),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(281),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(639),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(618),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(337),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(680),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalVelocity(ID1,50);
                               Dxl.setGoalVelocity(ID2,50);
                               Dxl.setGoalVelocity(ID3,50);
                               Dxl.setGoalPosition(ID1,convert_by_offset(668),UNIT_DEGREE);//-600 -200 -2500 PRENDI
                               Dxl.setGoalPosition(ID2,convert_by_offset(375),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(741),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID4,256);
                               delay(1000);
                               Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                               Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                               Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                               Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(94),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(501),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(390),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(284),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(653),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(548),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(343),UNIT_DEGREE);//200 600 -2400 LASCIA
                               Dxl.setGoalPosition(ID2,convert_by_offset(697),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(604),UNIT_DEGREE);                               
                               delay(1000);
                               Dxl.setGoalPosition(ID4,500);
                               delay(1000);
                               Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                               Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                               Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                               Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(542),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(147),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(287),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(683),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(337),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(463),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(733),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(390),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(510),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalVelocity(ID1,50);
                               Dxl.setGoalVelocity(ID2,50);
                               Dxl.setGoalVelocity(ID3,50);
                               Dxl.setGoalPosition(ID1,convert_by_offset(782),UNIT_DEGREE);//-200 -600 -2500 PRENDI
                               Dxl.setGoalPosition(ID2,convert_by_offset(431),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(568),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID4,256);
                               delay(1000);
                               Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                               Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                               Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                               Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(223),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(574),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(202),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(399),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(721),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(387),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(460),UNIT_DEGREE);//600 320 -2400 LASCIA
                               Dxl.setGoalPosition(ID2,convert_by_offset(762),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(445),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID4,500);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(223),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(574),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(202),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(399),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(721),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(387),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(460),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(762),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(445),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalVelocity(ID1,50);
                               Dxl.setGoalVelocity(ID2,50);
                               Dxl.setGoalVelocity(ID3,50);
                               Dxl.setGoalPosition(ID1,convert_by_offset(481),UNIT_DEGREE);//600 320 -2450 PRENDI
                               Dxl.setGoalPosition(ID2,convert_by_offset(791),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(463),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID4,256);
                               delay(1000);
                               Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                               Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                               Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                               Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(542),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(147),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(287),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(683),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(337),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(463),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(759),UNIT_DEGREE);//-200 -600 -2440 LASCIA
                               Dxl.setGoalPosition(ID2,convert_by_offset(404),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(536),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID4,500);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(94),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(501),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(390),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(284),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(653),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(548),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(343),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(697),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(604),UNIT_DEGREE);                               
                               delay(1000);
                               Dxl.setGoalVelocity(ID1,50);
                               Dxl.setGoalVelocity(ID2,50);
                               Dxl.setGoalVelocity(ID3,50);
                               Dxl.setGoalPosition(ID1,convert_by_offset(366),UNIT_DEGREE);//200 600 -2500 PRENDI
                               Dxl.setGoalPosition(ID2,convert_by_offset(718),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(624),UNIT_DEGREE);                               
                               delay(1000);
                               Dxl.setGoalPosition(ID4,256);
                               delay(1000);
                               Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                               Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                               Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                               Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(407),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(94),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(498),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(563),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(281),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(639),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(642),UNIT_DEGREE);//-600 -200 -2440 LASCIA
                               Dxl.setGoalPosition(ID2,convert_by_offset(349),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(709),UNIT_DEGREE);
                               delay(1000);
                               Dxl.setGoalPosition(ID4,500);
                               delay(1000);
                               Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                               Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                               delay(1000);
                               
                       }
                       }
                      else if (strcmp(demo,"DEM 3")) //DEMO 3, SPOSTA DUE OGGETTI IMPILATI (450 450), E LI IMPILA NELL'ALTRO PALLET (-450 -450). AL MOMENTO NON TORNA INDIETRO
                              
                              { 
                               for (k=0;k<nesecuzioni;k++)
                               {
                                 Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);  //zero
                                 Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(161),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(551),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(281),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(284),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(636),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(390),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalVelocity(ID1,50);
                                 Dxl.setGoalVelocity(ID2,50);
                                 Dxl.setGoalVelocity(ID3,50);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(340),UNIT_DEGREE);//ARRIVATO A 450 450 -2300, PRONTO PER PRELEVARE
                                 Dxl.setGoalPosition(ID2,convert_by_offset(683),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(442),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID4,256);
                                 delay(1000);
                                 Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                                 Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                                 Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(495),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(100),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(407),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(735),UNIT_DEGREE);  //ARRIVATO A -450 -450 -2450, PRONTO PER SGANCIARE
                                 Dxl.setGoalPosition(ID2,convert_by_offset(363),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(650),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID4,500);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(161),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(551),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(281),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(363),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(703),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(466),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalVelocity(ID1,50);
                                 Dxl.setGoalVelocity(ID2,50);
                                 Dxl.setGoalVelocity(ID3,50);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(434),UNIT_DEGREE);  //PRELEVA 2
                                 Dxl.setGoalPosition(ID2,convert_by_offset(771),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(545),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID4,256);
                                 delay(1000);
                                 Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                                 Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                                 Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(495),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(100),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(407),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(636),UNIT_DEGREE);  //SGANCIA 2 LO SGANCIA SPINTO TROPPO VERSO IL BASSO, NON E' IN LINEA DEL TUTTO CON QUELLO SOPRA
                                 Dxl.setGoalPosition(ID2,convert_by_offset(255),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(548),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID4,500);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(495),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(100),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(407),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(589),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(223),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(507),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalVelocity(ID1,50);
                                 Dxl.setGoalVelocity(ID2,50);
                                 Dxl.setGoalVelocity(ID3,50);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(636),UNIT_DEGREE); //TORNO A -450 -450 -2300 PER PRELEVARE
                                 Dxl.setGoalPosition(ID2,convert_by_offset(255),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(548),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID4,256);
                                 delay(1000);
                                 Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                                 Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                                 Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(161),UNIT_DEGREE);  //450 450 -2000
                                 Dxl.setGoalPosition(ID2,convert_by_offset(551),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(281),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(340),UNIT_DEGREE);  //450 450 -2300
                                 Dxl.setGoalPosition(ID2,convert_by_offset(683),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(442),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(422),UNIT_DEGREE);  //450 450 -2400 SGANCIO
                                 Dxl.setGoalPosition(ID2,convert_by_offset(753),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(521),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID4,500);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(495),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(100),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(407),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(636),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(255),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(548),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(697),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(340),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(621),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalVelocity(ID1,50);
                                 Dxl.setGoalVelocity(ID2,50);
                                 Dxl.setGoalVelocity(ID3,50);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(735),UNIT_DEGREE);  //-450 -450 -2450 PRELEVA 2
                                 Dxl.setGoalPosition(ID2,convert_by_offset(363),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(650),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID4,256);
                                 delay(1000);
                                 Dxl.setGoalVelocity(ID1,parameters_RAM[7]);
                                 Dxl.setGoalVelocity(ID2,parameters_RAM[7]);
                                 Dxl.setGoalVelocity(ID3,parameters_RAM[7]);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(161),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(551),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(281),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(340),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(683),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(442),UNIT_DEGREE);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID4,500);
                                 delay(1000);
                                 Dxl.setGoalPosition(ID1,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID2,convert_by_offset(0),UNIT_DEGREE);
                                 Dxl.setGoalPosition(ID3,convert_by_offset(0),UNIT_DEGREE);
                               }
             }
                             
           
            
      }
      

     
      
          void GRIP_ON_OF(const std_msg::Strin& string_grip)
      { 
                if (string_grip== "GON"))
            {
            Dxl.setGoalPosition(ID4, 256);
            
             }

               else if (string_grip=="GOF")
            {
            Dxl.setGoalPosition(ID4, 500);
            }
      
      }




void SSP(const std_msgs::String& info)
	{
		if (info=="DEGREE" ) //scrivo posizione in gradi
		{ 
			pos.x = Dxl.getPresentPosition(1,UNIT_DEGREE);
			pos.y = Dxl.getPresentPosition(2,UNIT_DEGREE);
			pos.z = Dxl.getPresentPosition(3,UNIT_DEGREE);
			pub_joint.publish(&pos);

		}
		
		else if (info=="RAW") //scrivo posizione in unità dynamixel
		{
			pos.x= Dxl.getPresentPosition(1,UNIT_RAW);
			pos.y= Dxl.getPresentPosition(2,UNIT_RAW);
			pos.z= Dxl.getPresentPosition(3,UNIT_RAW);
			pub_joint.publish(&pos);
		}
		
		if (info== "TORQUE")
			{
				if (Dxl.getTorqueEnableStat(1))
					{
					pub_trq.publish("TORQUE ON M1");
					}
			
				else
					{
					pub_trq.publish("NO TORQUE ON M1");
					}
			
			
				if (Dxl.getTorqueEnableStat(2))
					{
					pub_trq.publish("TORQUE ON M2");
					}
			
				else
					{
					pub_trq.publish("NO TORQUE ON M2");
					}
			
				if (Dxl.getTorqueEnableStat(3))
					{
					pub_trq.publish("TORQUE ON M3");
					}
			
				else
					{
					pub_trq.publish("NO TORQUE ON M3");
					}
			
			
	}


		if (info== "TEMP")
			{
			    temperatura1= Dxl.readControlTableItem(PRESENT_TEMPERATURE,1);
			    temperatura2= Dxl.readControlTableItem(PRESENT_TEMPERATURE,2);
			    temperatura3= Dxl.readControlTableItem(PRESENT_TEMPERATURE,3);
			    
			    pub_temp.publish(&temperatura1);
			    pub_temp.publish(&temperatura2);
			    pub_temp.publish(&temperatura3);
			}
			
	}





  
