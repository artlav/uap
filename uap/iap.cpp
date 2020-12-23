//#######################################################################################//           
//UAP core, Made by Artlav in 2011
//Autopilot class definition and utilitary functions
//#######################################################################################//
#define _CRT_SECURE_NO_DEPRECATE 1
//#######################################################################################//
#define _CRT_SECURE_NO_WARNINGS
#include <malloc.h>
#include <string.h>
#include "sal.h" 
#include "iap.h"
//#######################################################################################//
void iautopilot::reset()
{ 
 kp_pitch=-20;kp_roll=10;kp_yaw=10;
 spec_torque=tvec(1000,1000,1000);
 target_pitch=0;last_target_pitch=0;
 target_yaw=0;last_target_yaw=0;
 target_roll=0;last_target_roll=0;
 delta_cmd_pitch=0;delta_cmd_yaw=0;delta_cmd_roll=0;
 att_guide_not_set=1;
 rot_vel_bias=tvec(0,0,0);
 is_running=false;
 is_started=false;
 is_finished=false;
 is_loaded=false;
 is_failed=false;
 sprintf(state_string,"Running\0");
 sprintf(error,"Ok\0");
}
//#######################################################################################//
iautopilot::iautopilot(SAL *sal_in,VESSEL *vessel)
{
 us=vessel;
 sal=sal_in;
 sprintf(name,"Unnamed\0");
 var_cnt=0;
 vars=NULL;
 add_var("is_running",VT_INT,&is_running,VF_STATE,0,1);
 add_var("is_started",VT_INT,&is_started,VF_STATE,0,1);
 add_var("is_finished",VT_INT,&is_finished,VF_STATE,0,1);
 max_time_accel=1e50;
 att_split=0;
 reset();
}
iautopilot::~iautopilot(){}
//#######################################################################################//
void iautopilot::add_var(char *name,int typ,void *var,int flg,double min_val,double max_val)
{
 int c=var_cnt;
 var_cnt++;
 vars=(var_elem*)realloc(vars,sizeof(var_elem)*var_cnt);
 if(!vars)return;
 sprintf(vars[c].name,"%s\0",name);
 vars[c].typ=typ;
 vars[c].var=var;
 vars[c].flags=flg;
 vars[c].min_val=min_val;
 vars[c].max_val=max_val;
}
//#######################################################################################//
void iautopilot::set_dbl_var(char *name,double val)
{
 int i;
 for(i=0;i<var_cnt;i++)if(vars[i].typ==VT_DBL)if(!strcmp(vars[i].name,name)){
  if(val>vars[i].max_val)val=vars[i].max_val;
  if(val<vars[i].min_val)val=vars[i].min_val;
  *((double*)vars[i].var)=val;
  return;
 }
}
//#######################################################################################//
void iautopilot::set_int_var(char *name,int val)
{
 int i;
 for(i=0;i<var_cnt;i++)if(vars[i].typ==VT_INT)if(!strcmp(vars[i].name,name)){
  if(val>vars[i].max_val)val=(int)vars[i].max_val;
  if(val<vars[i].min_val)val=(int)vars[i].min_val;
  *((int*)vars[i].var)=val;
  return;
 }
}
//#######################################################################################//
void iautopilot::set_char_var(char *name,char *val)
{
 int i;
 for(i=0;i<var_cnt;i++)if(vars[i].typ==VT_CHAR)if(!strcmp(vars[i].name,name)){
  strncpy((char*)vars[i].var,val,(int)vars[i].max_val);
  return;
 }
}
//#######################################################################################//
void iautopilot::make_horvecs(VECTOR3 &wing,VECTOR3 &tail,VECTOR3 &nose,int grp)
{
 switch(grp){
  case xTHGROUP_RETRO:{
   wing=sal->horizon_rot(us,tvec(-1,0,0));
   tail=sal->horizon_rot(us,tvec(0,1,0));
   nose=sal->horizon_rot(us,tvec(0,0,-1));
   break;
  }
  case xTHGROUP_HOVER:{
   wing=sal->horizon_rot(us,tvec(1,0,0));
   tail=sal->horizon_rot(us,tvec(0,0,-1));
   nose=sal->horizon_rot(us,tvec(0,1,0));
   break;
  }
  case xTHGROUP_MAIN_INV:{
   wing=sal->horizon_rot(us,tvec(-1,0,0));
   tail=sal->horizon_rot(us,tvec(0,-1,0));
   nose=sal->horizon_rot(us,tvec(0,0,1));
   break;
  }
  default:{
   wing=sal->horizon_rot(us,tvec(1,0,0));
   tail=sal->horizon_rot(us,tvec(0,1,0));
   nose=sal->horizon_rot(us,tvec(0,0,1));
   break;
  }
 }
}  
//#######################################################################################//
void iautopilot::set_thgrp_level_addittive(double lv,THGROUP_TYPE g)
{
 int i,c;
 c=sal->group_thruster_count(us,g);
 for(i=0;i<c;i++){
  THRUSTER_HANDLE th=sal->group_thruster(us,g,i);
  if(th)if((sal->get_thruster_level(us,th)<lv))sal->set_thruster_level(us,th,lv);
 }
}
//#######################################################################################//
void iautopilot::set_rotation_rcs_levels(double pi,double yi,double ri,int grp)
{
 double p,y,r;
 if(grp==xTHGROUP_HOVER){
  p=pi;
  y=ri;
  r=-yi;
 }else if(grp==xTHGROUP_RETRO){
  p=-pi;
  y=yi;
  r=-ri;
 }else if(grp==xTHGROUP_MAIN_INV){
  p=-pi;
  y=-yi;
  r=ri;
 }else{
  p=pi;
  y=yi;
  r=ri;
 }
 
 if(!att_split){
  sal->set_thruster_group_level(us,THGROUP_ATT_YAWLEFT,0);
  sal->set_thruster_group_level(us,THGROUP_ATT_YAWRIGHT,0);
  sal->set_thruster_group_level(us,THGROUP_ATT_BANKRIGHT,0);
  sal->set_thruster_group_level(us,THGROUP_ATT_BANKLEFT,0);
  sal->set_thruster_group_level(us,THGROUP_ATT_PITCHUP,0);
  sal->set_thruster_group_level(us,THGROUP_ATT_PITCHDOWN,0);
 }

 set_thgrp_level_addittive(y>0?y:0,THGROUP_ATT_YAWLEFT);
 set_thgrp_level_addittive(y>0?0:-y,THGROUP_ATT_YAWRIGHT);
 set_thgrp_level_addittive(r>0?r:0,THGROUP_ATT_BANKRIGHT);
 set_thgrp_level_addittive(r>0?0:-r,THGROUP_ATT_BANKLEFT);
 set_thgrp_level_addittive(p>0?p:0,THGROUP_ATT_PITCHUP);
 set_thgrp_level_addittive(p>0?0:-p,THGROUP_ATT_PITCHDOWN);

 //Pitch: ^
 //Yaw: <
 //Roll: -v
}
//#######################################################################################//
double iautopilot::get_vertical_speed()
{
 VECTOR3 rv,rp;
 rv=sal->get_relative_vel(us,sal->get_gravity_ref(us));
 rp=sal->get_relative_pos(us,sal->get_gravity_ref(us));
 return smulv(nrvec(rv),nrvec(rp))*modv(rv);
}  
//#######################################################################################//
VECTOR3 iautopilot::get_angular_vel_grp(int grp)
{
 VECTOR3 a;
 a=sal->get_angular_vel(us)-rot_vel_bias;
 
      if(grp==xTHGROUP_MAIN)    return tvec(a.x,a.y,a.z);
 else if(grp==xTHGROUP_HOVER)   return tvec(a.x,-a.z,a.y);
 else if(grp==xTHGROUP_RETRO)   return tvec(-a.x,a.y,-a.z);
 else if(grp==xTHGROUP_MAIN_INV)return tvec(-a.x,-a.y,a.z);
                        else return tvec(a.x,a.y,a.z);
}
//#######################################################################################//
double iautopilot::get_bank(int grp)
{
 double sm,ac;
 VECTOR3 wing,tail,nose,ref;
 make_horvecs(wing,tail,nose,grp);
 
 ref=nrvec(tvec(0,1,0)-(nose*smulv(tvec(0,1,0),nose)));
 sm=smulv(ref,tail); 
 if(absd(sm)>=1)ac=0; else ac=sacos(sm);
 return sgn(wing.y)*ac;
}  
//#######################################################################################//
double iautopilot::get_yaw_vertical(int grp)
{
 double sm,ac;
 VECTOR3 wing,tail,nose,ref;
 make_horvecs(wing,tail,nose,grp);
 
 ref=nrvec(tvec(0,1,0)-((-tail)*smulv(tvec(0,1,0),(-tail))));
 sm=smulv(ref,nose);
 if(absd(sm)>=1)ac=0; else ac=sacos(sm);
 return -sgn(wing.y)*ac;
}  
//#######################################################################################//
double iautopilot::get_pitch(int grp)
{
 VECTOR3 wing,tail,nose;
 make_horvecs(wing,tail,nose,grp);
 return PI-PI/2-sacos(smulv(nose,tvec(0,1,0)));
}  
//#######################################################################################//
double iautopilot::get_pitch_horizon(int grp)
{
 double pit;
 VECTOR3 wing,tail,nose;
 make_horvecs(wing,tail,nose,grp);
 pit=sacos(tail.y);
 if(nose.y<0)pit=-pit;
 return pit;
}  
//#######################################################################################//
double iautopilot::get_slip_angle(int grp)
{
 double sm,ac;
 VECTOR3 wing,tail,nose;
 make_horvecs(wing,tail,nose,grp);
 
 VECTOR3 vv,vp,lv,hv,plv,pyw;
 vv=sal->get_relative_vel(us,sal->get_gravity_ref(us));
 vp=sal->get_global_pos(us);
 lv=sal->global_2_local(us,vp+vv);
 hv=nrvec(sal->horizon_rot(us,lv));
 
 plv=nrvec(hv-(tail*smulv(hv,tail)));
 pyw=nrvec(nose-(tail*smulv(nose,tail)));
 
 sm=smulv(plv,pyw);
 if(absd(sm)>=1)ac=0; else ac=sacos(sm);
 return -sgn(smulv(hv,wing))*ac;
}  
//#######################################################################################//
double iautopilot::get_heading(int grp)
{
 double hdg;
 VECTOR3 wing,tail,nose;
 make_horvecs(wing,tail,nose,grp);
 hdg=satan2(nose.x,nose.z);
 while(hdg<0)hdg+=2*PI;
 return hdg;
}
//#######################################################################################//
double iautopilot::get_roll_heading(int grp)
{
 double hdg;
 VECTOR3 wing,tail,nose;
 make_horvecs(wing,tail,nose,grp);
 hdg=-satan2(tail.x,-tail.z);
 while(hdg<0)hdg+=2*PI;
 return hdg;
}
//#######################################################################################//
double iautopilot::get_thruster_isp(THRUSTER_HANDLE th)
{
 OBJHANDLE pr=sal->thruster_fuel(us,th);
 if(pr)return sal->thruster_isp(us,th)*sal->fuel_efficiency(us,pr);
  else return sal->thruster_isp(us,th);
}
//#######################################################################################//
VECTOR3 iautopilot::get_group_thrust_vector(int tg,int ismax)
{
 THGROUP_TYPE g=sal->int2thgroup_type(tg);
 VECTOR3 res=tvec(0,0,0);
 VECTOR3 dir,cf,m;

 int c=sal->group_thruster_count(us,g);
 for(int i=0;i<c;i++){
  THRUSTER_HANDLE th=sal->group_thruster(us,g,i);
  if(th){
   dir=sal->thruster_dir(us,th);
   if(ismax)res+=dir*sal->thruster_max0(us,th);
   else{
    sal->thruster_moment(us,th,cf,m);
    res+=dir*modv(cf);
   }
  }
 }
 return res;
}
//#######################################################################################//
double iautopilot::get_group_thrust_angle(int tg,int ismax)
{
 THGROUP_TYPE g=sal->int2thgroup_type(tg);
 double tf=0,f=0,isp=0;
 VECTOR3 cf,m,ref,bal=tvec(0,0,0);
 
 int c=sal->group_thruster_count(us,g);
 for(int i=0;i<c;i++){
  THRUSTER_HANDLE th=sal->group_thruster(us,g,i);
  if(th){
   if(ismax)tf=sal->thruster_max0(us,th);else{
    sal->thruster_moment(us,th,cf,m);
    tf=modv(cf);
   }
   ref=sal->thruster_ref(us,th);
   if(modv(ref)>0)bal+=nrvec(ref)*tf;
  }
 }
 if(!(modv(bal)>0))return 0;
 switch(tg){
  case xTHGROUP_MAIN    :return -satan2(bal.y,-bal.z);
  case xTHGROUP_HOVER   :return -satan2(-bal.z,-bal.y);
  case xTHGROUP_RETRO   :return -satan2(bal.y,bal.z);
  case xTHGROUP_MAIN_INV:return -satan2(-bal.y,-bal.z);
  default:return -satan2(bal.y,-bal.z);
 }
}
//#######################################################################################//
double iautopilot::get_group_isp(int tg,int ismax)
{
 THGROUP_TYPE g=sal->int2thgroup_type(tg);
 double tf=0,f=0,isp=0;
 VECTOR3 cf,m;

 int c=sal->group_thruster_count(us,g);
 for(int i=0;i<c;i++){
  THRUSTER_HANDLE th=sal->group_thruster(us,g,i);
  if(th){
   if(ismax)tf=sal->thruster_max0(us,th);else{
    sal->thruster_moment(us,th,cf,m);
    tf=modv(cf);
   }
   f+=tf;
   isp+=get_thruster_isp(th)*tf;
  }
 }
 isp/=tf;
 return isp;
}
//#######################################################################################//
VECTOR3 iautopilot::get_major_gravity_local_acc_vector()
{
 VECTOR3 pp,vp,gx,dx;
 pp=sal->get_global_obj_pos(sal->get_surface_ref(us));
 vp=sal->get_global_pos(us);
 dx=nrvec(pp-vp);
 double dst=modv(pp-vp);
 if(dst<1.0)return tvec(0,0,0);   //FIXES crash, keeps problem?
 gx=dx*((GGRAV*sal->get_obj_mass(sal->get_surface_ref(us)))/(sqr(dst)));
 
 gx+=vp;
 return sal->global_2_local(us,gx);
}

//#######################################################################################//
double iautopilot::get_lin_max_accel()
{
 double tf,m,min_a,max_a;
 int i,j,c,tg;
 THGROUP_TYPE g;
 
 m=sal->get_ves_mass(us);
 min_a=1e10;
 max_a=0;
 
 for(j=0;j<32;j++){
  tg=xTHGROUP_ATT_ALL_LIN;
  tg&=1<<j;
  if(tg==0)continue;
  g=sal->int2thgroup_type(tg);
  
  tf=0;
  c=sal->group_thruster_count(us,g);
  if(c!=0){
   for(i=0;i<c;i++){
    THRUSTER_HANDLE th=sal->group_thruster(us,g,i);
    tf+=sal->thruster_max0(us,th);
   }
   if(tf/m>max_a)max_a=tf/m;
   if(tf/m<min_a)min_a=tf/m;
  }
 }

 return (min_a+max_a)/2;
}
//#######################################################################################//
double iautopilot::give_thgrp_accel(double acc,VECTOR3 dir,int groups)
{
 acc=sal->get_ves_mass(us)*acc;
 dir=nrvec(dir);
 VECTOR3 tv=dir*acc;
 double imodtv=modv(tv);
 int i;
 
 for(i=0;i<32;i++){
  int tg=groups&(1<<i);
  if(tg==0)continue;
  sal->set_thruster_group_level(us,sal->int2thgroup_type(tg),0);
 }

 for(i=0;i<32;i++){
  int tg=groups&(1<<i);
  if(tg==0)continue;
  THGROUP_TYPE g=sal->int2thgroup_type(tg);
  VECTOR3 tx=get_group_thrust_vector(tg,1);
  double nr=smulv(nrvec(tx),nrvec(tv));
  
  if(nr<=0.001){
   sal->set_thruster_group_level(us,g,0);
   continue;
  }
  
  VECTOR3 dx=nrvec(tx)*nr*modv(tv);
  double dl=modv(dx);
  double md=modv(tx);
  
  if(md>=dl){
   sal->set_thruster_group_level(us,g,dl/md);
   tv-=dx;
  }else{
   sal->set_thruster_group_level(us,g,1);
   tv-=dx;
  }
  if(modv(tv)/imodtv<=0.01)break;
  
 }

 return 0;
}
//#######################################################################################//
double iautopilot::set_thgrp_level(double lv,int groups)
{
 for(int i=0;i<32;i++){
  int tg=groups&(1<<i);
  if(tg==0)continue;
  sal->set_thruster_group_level(us,sal->int2thgroup_type(tg),lv);
 }
 return 0;
}
//#######################################################################################//
double iautopilot::get_thgrp_level(int groups)
{
 double m=0;
 int c=0;
 for(int i=0;i<32;i++){
  int tg=groups&(1<<i);
  if(tg==0)continue;
  m+=sal->get_thruster_group_level(us,sal->int2thgroup_type(tg));
  c++;
 }
 return m/c;
}
//#######################################################################################//
void iautopilot::stop_all_engines()
{
 sal->set_thruster_group_level(us,THGROUP_MAIN,0.0);
 sal->set_thruster_group_level(us,THGROUP_RETRO,0.0);
 sal->set_thruster_group_level(us,THGROUP_HOVER,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_PITCHUP,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_PITCHDOWN,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_YAWLEFT,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_YAWRIGHT,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_BANKLEFT,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_BANKRIGHT,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_RIGHT,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_LEFT,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_UP,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_DOWN,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_FORWARD,0.0);
 sal->set_thruster_group_level(us,THGROUP_ATT_BACK,0.0);
}
//#######################################################################################//
void iautopilot::att_get_specific_torque(int grp)
{
 int nThrusters=sal->thruster_count(us);
 double P,Y,R;
 
 P=sal->get_thruster_group_level(us,THGROUP_ATT_PITCHUP)-sal->get_thruster_group_level(us,THGROUP_ATT_PITCHDOWN);
 Y=sal->get_thruster_group_level(us,THGROUP_ATT_YAWRIGHT)-sal->get_thruster_group_level(us,THGROUP_ATT_YAWLEFT);
 R=sal->get_thruster_group_level(us,THGROUP_ATT_BANKRIGHT)-sal->get_thruster_group_level(us,THGROUP_ATT_BANKLEFT);

 VECTOR3 TotalM=tvec(0,0,0);
 for(int i=0;i<nThrusters;i++){
  VECTOR3 F,M;
  sal->thruster_moment(us,sal->thruster_by_index(us,i),F,M);
  TotalM=TotalM+M;
 }
 if(P!=0)spec_torque.x=absd(TotalM.x/P);
 if(Y!=0)spec_torque.y=absd(TotalM.y/Y);
 if(R!=0)spec_torque.z=absd(TotalM.z/R);
}
//#######################################################################################//
void iautopilot::asf_get_specific_torque(int grp)
{
 //FIXME: ???
 spec_torque=tvec(80000,80000,40000);
}
//#######################################################################################//
void iautopilot::att_guide(double dt,double tgt_pitch,double tgt_yaw,double tgt_roll,int grp)
{
 VECTOR3 vec,pmi;
 
 last_target_pitch=target_pitch;
 last_target_yaw=target_yaw;
 last_target_roll=target_roll;
 target_pitch=tgt_pitch;
 target_yaw=tgt_yaw;  
 target_roll=tgt_roll;
 if(att_guide_not_set){
  last_target_pitch=target_pitch;
  last_target_yaw=target_yaw;
  last_target_roll=target_roll;
  att_guide_not_set=0;
 }
 
 vec=sal->get_ves_pmi(us);
 pmi=vec*sal->get_ves_mass(us);
 att_get_specific_torque(grp);
 
 if(grp==xTHGROUP_HOVER){ 
  kd_pitch=calc_kd(spec_torque.x,pmi.x,kp_pitch);
  kd_yaw  =calc_kd(spec_torque.z,pmi.z,kp_yaw);
  kd_roll =calc_kd(spec_torque.y,pmi.y,kp_roll); 
 }else{ 
  kd_pitch=calc_kd(spec_torque.x,pmi.x,kp_pitch);
  kd_yaw  =calc_kd(spec_torque.y,pmi.y,kp_yaw);
  kd_roll =calc_kd(spec_torque.z,pmi.z,kp_roll); 
 }
 
 delta_cmd_pitch=(target_pitch-last_target_pitch)/dt;
 delta_cmd_yaw  =(target_yaw  -last_target_yaw  )/dt;
 delta_cmd_roll =(target_roll -last_target_roll )/dt;
}
//#######################################################################################//
void iautopilot::att_control_exp(double dt,double pitch,double yaw,double roll,double tgt_pitch,double tgt_yaw,double tgt_roll,int flg,int grp)
{
 VECTOR3 ang_vel;
 double pitchcmd,yawcmd,rollcmd,p,d,p_set,y_set,r_set;
 
 pitch=pitch -tgt_pitch;while(pitch<-PI)pitch+=2*PI;while(pitch>PI)pitch-=2*PI;
 yaw  =yaw   -tgt_yaw  ;while(yaw  <-PI)yaw  +=2*PI;while(yaw  >PI)yaw  -=2*PI; 
 roll =roll  -tgt_roll ;while(roll <-PI)roll +=2*PI;while(roll >PI)roll -=2*PI;
 
 ang_vel=get_angular_vel_grp(grp);

 p_set=(delta_cmd_pitch*dt);p=(pitch-p_set);d=( ang_vel.x-delta_cmd_pitch);pitchcmd=kp_pitch*p+kd_pitch*d;
 y_set=(delta_cmd_yaw  *dt);p=(yaw  -y_set);d=(-ang_vel.y-delta_cmd_yaw  );yawcmd  =kp_yaw  *p+kd_yaw  *d;
 r_set=(delta_cmd_roll *dt);p=(roll -r_set);d=(-ang_vel.z-delta_cmd_roll );rollcmd =kp_roll *p+kd_roll *d;
 
 //sprintf(oapiDebugString(),"target_pitch=%f p_set=%f pitch=%f pitch_cmd=%",target_pitch/RAD,p_set/RAD,pitch/RAD,pitchcmd);
 //sprintf(oapiDebugString(),"target_yaw=%f y_set=%f yaw=%f yawcmd=%f",target_yaw/RAD,y_set/RAD,yaw/RAD,yawcmd);
 //sprintf(oapiDebugString(),"target_roll=%f CmdR=%f roll=%f P=%f delta_cmd_roll=%f ang_vel.z=%f D=%f",target_roll/RAD,CmdR/RAD,roll/RAD,P/RAD,delta_cmd_roll/RAD,ang_vel.z/RAD,D/RAD);
 
 set_rotation_rcs_levels(flg&0x1?pitchcmd:0,flg&0x2?yawcmd:0,flg&0x4?rollcmd:0,grp);
}
//#######################################################################################//
void iautopilot::att_killrot(int flg,int grp)
{
 VECTOR3 ang_vel;
 double pitchcmd,yawcmd,rollcmd;
 att_guide_not_set=1;
 att_guide(1,0,0,0,grp);
 
 ang_vel=get_angular_vel_grp(grp);
 pitchcmd= kd_pitch*ang_vel.x;
 yawcmd  =-kd_yaw  *ang_vel.y;
 rollcmd =-kd_roll *ang_vel.z;
 
 set_rotation_rcs_levels(flg&0x1?pitchcmd:0,flg&0x2?yawcmd:0,flg&0x4?rollcmd:0,grp);
}
//#######################################################################################//
void iautopilot::att_control(double dt,double tgt_pitch,double tgt_yaw,double tgt_roll,int grp)
{
 att_control_exp(dt,get_pitch(grp),get_heading(grp),get_bank(grp),tgt_pitch,tgt_yaw,tgt_roll,7,grp);
}
//#######################################################################################//
//#######################################################################################//
void iautopilot::asf_guide(double dt,double tgt_pitch,double tgt_yaw,double tgt_roll,int grp)
{
 VECTOR3 vec,pmi;
 
 last_target_pitch=target_pitch;
 last_target_yaw=target_yaw;
 last_target_roll=target_roll;
 target_pitch=tgt_pitch;
 target_yaw=tgt_yaw;  
 target_roll=tgt_roll;
 if(att_guide_not_set){
  last_target_pitch=target_pitch;
  last_target_yaw=target_yaw;
  last_target_roll=target_roll;
  att_guide_not_set=0;
 }
 
 vec=sal->get_ves_pmi(us);
 pmi=vec*sal->get_ves_mass(us);
 asf_get_specific_torque(grp);
 
 kd_pitch=calc_kd(spec_torque.x,pmi.x,kp_pitch);
 kd_yaw  =calc_kd(spec_torque.y,pmi.y,kp_yaw);
 kd_roll =calc_kd(spec_torque.z,pmi.z,kp_roll); 

 delta_cmd_pitch=(target_pitch-last_target_pitch)/dt;
 delta_cmd_yaw  =(target_yaw  -last_target_yaw  )/dt;
 delta_cmd_roll =(target_roll -last_target_roll )/dt;
}
//#######################################################################################//
void iautopilot::asf_control_exp(double dt,double pitch,double yaw,double roll,double tgt_pitch,double tgt_yaw,double tgt_roll,int flg,int grp)
{
 VECTOR3 ang_vel;
 double pitchcmd,yawcmd,rollcmd,p,d,p_set,y_set,r_set;
 
 yaw=yaw-tgt_yaw;
 while(yaw<-PI)yaw+=2*PI;
 while(yaw> PI)yaw-=2*PI;
 
 ang_vel=get_angular_vel_grp(grp);

 p_set=(tgt_pitch+delta_cmd_pitch*dt);p=(pitch-p_set);d=( ang_vel.x-delta_cmd_pitch);pitchcmd=kp_pitch*p+kd_pitch*d;
 y_set=(          delta_cmd_yaw  *dt);p=(yaw  -y_set);d=(-ang_vel.y-delta_cmd_yaw  );yawcmd  =kp_yaw  *p+kd_yaw  *d;
 r_set=(tgt_roll +delta_cmd_roll *dt);p=(roll -r_set);d=(-ang_vel.z-delta_cmd_roll );rollcmd =kp_roll *p+kd_roll *d;
 
 //sprintf(oapiDebugString(),"target_pitch=%f CmdP=%f pitch=%f P=%f delta_cmd_pitch=%f ang_vel.x=%f D=%f",target_pitch/RAD,CmdP/RAD,pitch/RAD,P/RAD,delta_cmd_pitch/RAD,ang_vel.x/RAD,D/RAD);
 //sprintf(oapiDebugString(),"target_yaw=%f CmdY=%f yaw=%f P=%f delta_cmd_yaw=%f ang_vel.y=%f D=%f",target_yaw/RAD,CmdY/RAD,yaw/RAD,P/RAD,delta_cmd_yaw/RAD,ang_vel.y/RAD,D/RAD);
 //sprintf(oapiDebugString(),"target_roll=%f CmdR=%f roll=%f P=%f delta_cmd_roll=%f ang_vel.z=%f D=%f",target_roll/RAD,CmdR/RAD,roll/RAD,P/RAD,delta_cmd_roll/RAD,ang_vel.z/RAD,D/RAD);
 
 if(flg&0x1)sal->set_airsurface_level(us,AIRCTRL_ELEVATOR,pitchcmd);
 if(flg&0x2)sal->set_airsurface_level(us,AIRCTRL_RUDDER,yawcmd);
 if(flg&0x4)sal->set_airsurface_level(us,AIRCTRL_AILERON,rollcmd);
}
//#######################################################################################//
void iautopilot::asf_control(double dt,double tgt_pitch,double tgt_yaw,double tgt_roll,int grp)
{
 asf_control_exp(dt,get_pitch(grp),get_heading(grp),get_bank(grp),tgt_pitch,tgt_yaw,tgt_roll,7,grp);
}
//#######################################################################################//
void iautopilot::asf_zero()
{
 sal->set_airsurface_level(us,AIRCTRL_ELEVATOR,0);
 sal->set_airsurface_level(us,AIRCTRL_RUDDER,0);
 sal->set_airsurface_level(us,AIRCTRL_AILERON,0);
}
//#######################################################################################//