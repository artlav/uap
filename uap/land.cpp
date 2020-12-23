//#######################################################################################//            
//UAP module, Made by Artlav in 2011
//Landing autopilot
//#######################################################################################//
#define _CRT_SECURE_NO_DEPRECATE 1
//#######################################################################################//
#include "sal.h"
#include "iap.h"
#include "land.h"
//#######################################################################################//
ap_get_on_pad::ap_get_on_pad(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Land on pad\0");
 add_var("engine",VT_INT,&use_grp,0,1,15);
 add_var("tgt_lat",VT_DBL,&tgt_lat,VF_DEG,-360*RAD,360*RAD);
 add_var("tgt_lon",VT_DBL,&tgt_lon,VF_DEG,-360*RAD,360*RAD);
 
 sal->get_ves_equ_pos(us,tgt_lon,tgt_lat,tgt_rad);
 max_time_accel=10;
 use_grp=xTHGROUP_HOVER;
}
//#######################################################################################//
void ap_get_on_pad::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 init();
}
//#######################################################################################//
void ap_get_on_pad::init()
{
 stage=0;
 is_loaded=true;
 
 stop_all_engines();
 
 tgt_rad=sal->get_obj_size(sal->get_gravity_ref(us));
 
 dist=2*sal->get_ves_size(us);
 
 set_a=1;
 set_ns=20.0;
 set_ang=40*RAD;
 
 sprintf(state_string,"Approaching");
}
//#######################################################################################//
void ap_get_on_pad::stop()
{
 iautopilot::stop();
 stop_all_engines();
}
//#######################################################################################//
void ap_get_on_pad::step(double simt,double simdt)
{
 if(!is_loaded)init();
 if(is_running){
  if(sal->ves_ground_contact(us)){stop();return;}

  VECTOR3 pyr,rp,rv,tp,vp,nv,dv,av,pp;
  double p,y,t,a,ns,c;
  
  a=set_a;
  ns=set_ns;
  tgt_pos=sal->sph_2_rec(sal->get_gravity_ref(us),tgt_lat,tgt_lon,tgt_rad);
  
  pp=sal->get_global_obj_pos(sal->get_gravity_ref(us));
  vp=sal->get_global_pos(us);
  tp=pp+tgt_pos+nrvec(tgt_pos)*dist;
  rp=sal->global_2_local(us,tp);
  
  if(modv(rp)>1000){
   ns=set_ns*9.5;
   set_ang=50*RAD;
  }
  if(modv(rp)<9){
   sprintf(state_string,"Careful");
   ns=set_ns/2;
   set_ang=10*RAD;
  }
  if(modv(rp)<1){
   sprintf(state_string,"Padding");
   dist=0;
   ns=set_ns/4;
   set_ang=5*RAD;
  }
  
  rv=sal->get_ves_airspeed_vector(us);
 
  t=modv(rv)/a;
  if(ns*t>modv(rp))ns=modv(rp)/t;
  nv=nrvec(rp)*ns;
  dv=nv-rv;
  if(a*simdt>modv(dv))a=modv(dv)/simdt;
  
  av=-get_major_gravity_local_acc_vector();
  av=av+(nrvec(av)*smulv(av,dv)*modv(dv));
   
  //sprintf(oapiDebugString(),"rv=%f,%f,%f rp=%f,%f,%f",rv.x,rv.y,rv.z,rp.x,rp.y,rp.z);
  //sprintf(oapiDebugString(),"a=%f ns=%f rv=%f,%f,%f dv=%f,%f,%f nv=%f,%f,%f",a,ns,rv.x,rv.y,rv.z,dv.x,dv.y,dv.z,nv.x,nv.y,nv.z);
  //sprintf(oapiDebugString(),"pyr=%f,%f,%f",pyr.x,pyr.y,pyr.z);
  //sprintf(oapiDebugString(),"ns=%f",ns);
  
  if(modv(dv)>set_ns*10){
   pyr=get_pitchyawroll(nrvec(dv),tvec(0,0,0));
   set_thgrp_level(1,xTHGROUP_MAIN);
   att_guide(simdt,0,0,0,xTHGROUP_MAIN);
   att_control_exp(simdt,pyr.x,pyr.y,pyr.z,0,0,0,3,xTHGROUP_MAIN);
  }else{
   set_thgrp_level(0,xTHGROUP_MAIN);
   c=-dv.z/ns;
   if(c>1)c=1;
   if(c<-1)c=-1;
   p=90*RAD+set_ang*c;
   c=dv.x/ns;
   if(c>1)c=1;
   if(c<-1)c=-1;
   y=set_ang*c;
   give_thgrp_accel(modv(av),av,use_grp);
   att_guide(simdt,p,y,0,use_grp);
   att_control_exp(simdt,get_pitch_horizon(use_grp),get_yaw_vertical(use_grp),0,p,y,0,7,use_grp);
  }

  
  /*
  
  //RCS
  VECTOR3 rp,rv,tp,vp,nv,dv,av,pp;
  double t,a,ns,c;
  
  simdt=simdt*2;
  a=set_a;
  ns=set_ns;
  EquToRel(us,tgt_lat,tgt_lon,tgt_rad,tgt_pos);
  
  get_global_obj_pos(sal->get_gravity_ref(us),&pp);
  sal->get_global_pos(us,vp);
  tp=pp+tgt_pos+nrvec(tgt_pos)*dist;
  sal->global_2_local(us,tp,rp);
  
  if(modv(rp)<0.1){
   dist=0;
   set_ns=set_ns/4;
  }
  
  get_ves_airspeed_vector(us,rv);
 
  t=modv(rv)/a;
  if(ns*t>modv(rp))ns=modv(rp)/t;
  nv=nrvec(rp)*ns;
  dv=nv-rv;
  if(a*simdt>modv(dv))a=modv(dv)/simdt;
  
  av=-get_major_gravity_local_acc_vector();
  c=(smulv(nrvec(av),nrvec(nv))*modv(nv));
  if(c>=0)c=1;
  if(c<0)c=0;
  c=1;
  
  dv=nrvec(dv)*a+av*c;
  a=modv(dv);
  
  //sprintf(oapiDebugString(),"rv=%f,%f,%f rp=%f,%f,%f",rv.x,rv.y,rv.z,rp.x,rp.y,rp.z);
  sprintf(oapiDebugString(),"a=%f ns=%f rv=%f,%f,%f dv=%f,%f,%f nv=%f,%f,%f",a,ns,rv.x,rv.y,rv.z,dv.x,dv.y,dv.z,nv.x,nv.y,nv.z);
  //sprintf(oapiDebugString(),"pyr=%f,%f,%f",pyr.x,pyr.y,pyr.z);
  //sprintf(oapiDebugString(),"ns=%f",ns);
  
  if(stp==0){
   give_thgrp_accel(a,dv,xTHGROUP_ATT_ALL_LIN+use_grp);
   stp=1;
  }else{
   att_guide(simdt,90*RAD,0,0,use_grp);
   att_control_exp(simdt,get_pitch_horizon(use_grp),get_yaw_vertical(use_grp),0,90*RAD,0,0,7,use_grp);
   stp=0;
  }
  */
 }
}
//#######################################################################################//
iautopilot *mk_get_on_pad(SAL *sal_in,VESSEL *v){return new ap_get_on_pad(sal_in,v);}
//#######################################################################################//