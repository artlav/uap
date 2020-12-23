//#######################################################################################//      
//UAP module, Made by Artlav in 2011
//Docking autopilot
//#######################################################################################//
#define _CRT_SECURE_NO_DEPRECATE
//#######################################################################################//
#include "sal.h"
#include "iap.h"
#include "dock.h"
//#######################################################################################//
ap_dock::ap_dock(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Docking\0");
 add_var("target",VT_CHAR,tgt_nam,0,0,255);
 add_var("port",VT_INT,&port,0,0,255);
 add_var("with_port",VT_INT,&with_port,0,0,255);
 add_var("distance",VT_DBL,&distance,VF_OUT,0,10000);
 
 sprintf(tgt_nam,"ISS\0");
 port=0;
 with_port=0;
 max_time_accel=10;
 att_split=1;
 distance=0;
}
//#######################################################################################//
void ap_dock::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 init();
}
//#######################################################################################//
void ap_dock::init()
{
 stage=0;
 is_loaded=true;
 
 stop_all_engines();
 
 tgt_ves=sal->vessel_by_name(tgt_nam);
 if(!tgt_ves){stop();return;}
 
 tgt_dock=sal->get_dock(tgt_ves,port);
 our_dock=sal->get_dock(us,with_port);
 if(!tgt_dock){stop();return;}
 if(!our_dock){stop();return;}
 if(sal->dock_status(us,with_port)){stop();return;}
 if(sal->dock_status(tgt_ves,port)){stop();return;}
 
 dist=sal->get_ves_size(us);
 
 set_a=get_lin_max_accel();
 if(set_a<=0)set_a=0.1;
 set_ns=10*set_a;
 if(set_ns<1.0)set_ns=1.0;
 sprintf(state_string,"Getting closer");
}
//#######################################################################################//
void ap_dock::stop()
{
 iautopilot::stop();
 stop_all_engines();
}
//#######################################################################################//
void ap_dock::step(double simt,double simdt)
{
 if(!is_loaded)init();
 if(is_running){
  if((sal->dock_status(us,with_port))&&(sal->dock_status(tgt_ves,port))){stop();return;} 
 
  VECTOR3 pyr,p,d,r,up,ud,ur,rp,rv,tv,tp,vv,vp,nv,dv;
  double t,a,ns;
  
  a=set_a;
  ns=set_ns;
  
  vp=sal->get_global_pos(us);
  vv=sal->get_global_vel(us);
  tp=sal->get_global_pos(tgt_ves);
  tv=sal->get_global_vel(tgt_ves);
  
  sal->dock_params(us,our_dock,up,ud,ur);
  sal->dock_params(tgt_ves,tgt_dock,p,d,r);
  rp=p+d*dist;
  rp=sal->local_2_global(tgt_ves,rp);
  rp=sal->global_2_local(us,rp);
  
  rp=rp-up;
  
  MATRIX3 xmat=vecs2mat_orb(ud,ur);
  r=lvmat(xmat,r);
  d=lvmat(xmat,-d);
  
  if((absd(xmat.data[0]-1)>0.001)||(absd(xmat.data[4]-1)>0.001)||(absd(xmat.data[8]-1)>0.001))if((r.y<0)||(r.z<0)||(d.y<0)){r=-r;d=-d;}//FIXME: Grand orientation issues
  
  r=sal->global_2_local(us,sal->local_2_global(tgt_ves,r)-tp+vp); 
  d=sal->global_2_local(us,sal->local_2_global(tgt_ves,d)-tp+vp); 
  //sprintf(oapiDebugString(),"d=%f %f %f r=%f %f %f",d.x,d.y,d.z,r.x,r.y,r.z);
  pyr=get_pitchyawroll(d,r);
  
  
  rot_vel_bias=sal->get_angular_vel(tgt_ves);
  
  if((dist==0)||(modv(rp)<0.5)){
   sprintf(state_string,"Coming in");
   dist=0;
   ns=set_ns/4;
  }
  
  if(modv(rp)>10000){fail_error("Too far away");return;}
  
  rv=vp+tv-vv;
  rv=sal->global_2_local(us,rv);
 
  
  t=modv(rv)/a;
  if(ns*t>modv(rp))ns=modv(rp)/t;
  nv=nrvec(rp)*ns;
  dv=nv+rv;
  if(a*simdt>modv(dv))a=modv(dv)/simdt;
  
  if(modv(dv)>50){fail_error("Relative velocity is too fast");return;}
  
  distance=modv(rp);    
  if((distance<0.25)&&(modv(dv)<0.1)){sprintf(state_string,"Floating");stop_all_engines();return;} 
  
  stop_all_engines();
  give_thgrp_accel(a,dv,xTHGROUP_ATT_ALL_LIN);
  att_guide(simdt,0,0,0,xTHGROUP_MAIN);
  att_control_exp(simdt,pyr.x,pyr.y,pyr.z,0,0,0,7,xTHGROUP_MAIN);
 }
}
//#######################################################################################//
iautopilot *mk_dock(SAL *sal_in,VESSEL *v){return new ap_dock(sal_in,v);}
//#######################################################################################//