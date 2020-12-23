//#######################################################################################//
//UAP module, Made by Artlav in 2011
//Launch autopilot
//Based on OrbiterPEG by Chris Jeppesen, 2007
//#######################################################################################//
#define _CRT_SECURE_NO_DEPRECATE 1
//#######################################################################################//
//#######################################################################################// 
#include "sal.h"
#include "iap.h"
#include "peg.h"
//#######################################################################################//
ap_transorbit::ap_transorbit(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Trans-orbit\0"); 
 add_var("engine",VT_INT,&use_grp,0,1,15);
 add_var("kind",VT_INT,&kind,0,0,2);
 add_var("target",VT_CHAR,target,0,0,255);
 add_var("heading",VT_DBL,&tgt_az,VF_DEG,-2,360*RAD);
 add_var("apoapsis",VT_DBL,&tgt_apoapsis,0,0,1e10);
 add_var("periapsis",VT_DBL,&tgt_periapsis,0,0,1e10);
 add_var("ta",VT_DBL,&tgt_ta,VF_DEG,0,360*RAD);
 add_var("mode",VT_INT,&mode,VF_STATE,0,10);
 add_var("kind_is",VT_CHAR,kind_is,VF_OUT,0,255);
  
 tgt_periapsis=200000;
 tgt_apoapsis=200000;
 tgt_ta=0;
 target[0]=0;
 tgt_az=90*RAD;
 
 stop_dt=3;
 
 use_grp=xTHGROUP_MAIN;
 
 kind=0;
 use_target=0;

 rmh=tvec(1,0,0); //FIXME: WTF is it?
 
 reset();
}
//#######################################################################################//
void ap_transorbit::start()
{
 iautopilot::start();
 is_started=true;
}
//#######################################################################################//
void ap_transorbit::reset()
{
 iautopilot::reset();
 mode=0;
 max_time_accel=10;
}
//#######################################################################################//
void ap_transorbit::info()
{
 if((kind==1)||(kind==2))if(!is_running)tgt_periapsis=sal->get_ves_altitude(us);
 if(kind==2)if(!is_running)tgt_apoapsis=sal->get_ves_altitude(us);
 switch(kind){
  case 0:set_char_var("kind_is","Bi-apsis");break;
  case 1:set_char_var("kind_is","Mono-apsis");break;
  case 2:set_char_var("kind_is","Circularize");break;
  default:set_char_var("kind_is","WTF?");
 }
}
//#######################################################################################//
void ap_transorbit::init(double ct)
{
 is_running=true;
 is_loaded=true;
 no_guidance_computed=true;
 
 if(kind==1){tgt_periapsis=sal->get_ves_altitude(us);}
 if(kind==2){tgt_periapsis=sal->get_ves_altitude(us);tgt_apoapsis=tgt_periapsis;}
 
 tgt_obj=0;
 use_target=true;
 if(target[0]!=0)tgt_obj=sal->object_by_name(target);
 if(!tgt_obj)use_target=false;
 
 att_guide_not_set=1;
 a0=0;
 tau=0;
 last_calc=0.0;
 calc_step=0.5;
 t_cutoff=145;
 thrust=0;
 
 t0=ct;
 rh0=sal->get_relative_pos(us,sal->get_gravity_ref(us));
 rh0=nrvec(rh0);
 mu=GGRAV*sal->get_obj_mass(sal->get_gravity_ref(us));
 
 set_thgrp_level(1,use_grp);
 
 sprintf(state_string,"Going up");
}
//#######################################################################################//
void ap_transorbit::stop()
{
 iautopilot::stop();
 is_running=false;
 is_started=false;
 is_finished=true;
 stop_all_engines();
 mode=4;
}
//#######################################################################################//
void ap_transorbit::navigate()
{
 double ref_radius;
 VECTOR3 pos,vel,hv,rh;
 
 pos=sal->get_relative_pos(us,sal->get_gravity_ref(us));
 vel=sal->get_relative_vel(us,sal->get_gravity_ref(us));
 ref_radius=sal->get_obj_size(sal->get_gravity_ref(us));
 
 hv=vmulv(pos,vel);
 rh=nrvec(pos);
 
 tgt_rad_periapsis=tgt_periapsis+ref_radius;
 tgt_rad_apoapsis=tgt_apoapsis+ref_radius;
 
 rT=tgt_rad_periapsis;
 if(kind==1){tgt_periapsis=sal->get_ves_altitude(us);rT=tgt_rad_periapsis;}
 if(kind==2){tgt_periapsis=sal->get_ves_altitude(us);rT=tgt_rad_periapsis;tgt_apoapsis=tgt_periapsis;}

 tgt_ecc=(tgt_rad_apoapsis-tgt_rad_periapsis)/(tgt_rad_periapsis+tgt_rad_apoapsis);
 ref_dst=modv(pos); 
 vr=smulv(vel,rh);
 h=modv(hv); 
 omega=smulv(vel,vmulv(nrvec(hv),rh))/ref_dst;
 
 
 isp=get_group_isp(use_grp,0);
 thrust=modv(get_group_thrust_vector(use_grp,0));
 thr_angle=get_group_thrust_angle(use_grp,0);
 if(thrust<=0){  //If no thrust now,get the max thrust
  isp=get_group_isp(use_grp,1);
  thrust=modv(get_group_thrust_vector(use_grp,1));
  thr_angle=get_group_thrust_angle(use_grp,1);
 }
 if(thrust<1)return;
 
 OBJHANDLE ref;
 ref=sal->get_gravity_ref(us);
 if(use_target)tgt_az=sal->calculate_azimuth(us,sal->get_obj_size(ref)+sal->get_obj_altitude(tgt_obj,ref),sal->get_obj_equ_inclination(tgt_obj,ref)); 
 if(use_target)if(tgt_az<0)tgt_az=2*PI+tgt_az;

 m=sal->get_ves_mass(us);
 a0=thrust/m;
 tau=isp/a0;
 
 theta=sacos(smulv(rh0,rh));
 phi=sacos(smulv(rh,rmh));
 VECTOR3 rhxrmh=vmulv(rh,rmh);
 if(rhxrmh.y>0)phi=2*PI-phi;
}
//#######################################################################################//
int c=0;
void ap_transorbit::estimate(double ct)
{
 double num,fh,fdoth;   
 ref_dst=max(ref_dst,0.1); //Could get to be zero   
                    
 rbar=(ref_dst+rT)/2;
 num=mu/sqr(rbar)-sqr(omega)*ref_dst;
 fr=A+num/a0;
 fdotr=B+(num/a(t_cutoff)-fr)/t_cutoff;
 fh=0;   //No yaw guidance yet
 fdoth=0;
 ftheta=1-fr*fr/2-fh*fh/2;  //Small number approximation to hypot
 fdottheta=-(fr*fdotr+fh*fdoth);
 fdotdottheta=-(fdotr*fdotr+fdoth*fdoth)/2;  

 //Estimate true anomaly at cutoff
 d3=h*vr/(ref_dst*ref_dst*ref_dst);  
 d4=(h*vrT/(rT*rT*rT)-d3)/t_cutoff;             
 deltatheta=(h/sqr(ref_dst))*t_cutoff+(ftheta*c0(t_cutoff)+fdottheta*cn(t_cutoff,1)+fdotdottheta*cn(t_cutoff,2))/rbar-d3*sqr(t_cutoff)-d4*t_cutoff*t_cutoff*t_cutoff/3.0;
 thetaT=theta+deltatheta;
 if(tgt_ecc==0)tgt_ta=0;

 //Calculate orbit parameters for this ta
 if(tgt_ecc!=1)p=(tgt_rad_periapsis/(1-tgt_ecc))*(1-tgt_ecc*tgt_ecc);
          else p=2*tgt_rad_periapsis;
                      

 //Estimate time of cutoff
 deltah=sqsrt(mu*p)-h;
 deltav=deltah/rbar;
 t_cutoff=tau*(1-exp(-deltav/isp));
 if(t_cutoff>1000)t_cutoff=1000;
 if((tau<=t_cutoff)||(t_cutoff<=0))t_cutoff=tau-stop_dt;
 if(deltav<-1)t_cutoff=0;
 
 
 stop_met=t_cutoff+ct;
 //Estimate radius at cutoff
 rT=p/(1+tgt_ecc*cos(tgt_ta));
 //estimate vertical speed at cutoff
 vrT=sqsrt(mu/p)*tgt_ecc*sin(tgt_ta);   
}
//#######################################################################################//
void ap_transorbit::guide(double ct)
{
 double tgt_pitch,tgt_yaw,tgt_roll;
 
 if(t_cutoff>stop_dt){
  double a=b0(t_cutoff);
  double b=bn(t_cutoff,1);
  double c=c0(t_cutoff);
  double d=cn(t_cutoff,1);
  double y1=vrT-vr;
  double y2=rT-vr*t_cutoff-ref_dst;
  double D=a*d-b*c;
  A=(d*y1-b*y2)/D;
  B=(a*y2-c*y1)/D;
  last_control_upd=ct;
 }
  
 C=(mu/sqr(ref_dst)-sqr(omega)*ref_dst)/a0;
 tgt_roll=0;
 tgt_yaw=0;
 
 mode=t_cutoff>stop_dt?1:2;
 fhdotrh=A+B*(ct-last_control_upd)+C;

 if(absd(fhdotrh)>1){
  if(mode!=3)att_guide_not_set=1;
  mode=3;
  double lv=get_thgrp_level(use_grp);
  if(C<0.5)set_thgrp_level(lv/2<0.1?0.1:lv/2,use_grp);
      else set_thgrp_level(lv*2>1.0?1.0:lv*2,use_grp);
  pitch_on=true;
  tgt_pitch=get_pitch_horizon(use_grp);
  sprintf(state_string,"Lost, going blind");
 }else{
  if(mode==3)att_guide_not_set=1;
  mode=t_cutoff>stop_dt?1:2;
  pitch_on=true;
  tgt_pitch=PI/2-sacos(fhdotrh);
  sprintf(state_string,"Navigating");
 }
 yaw_on=true;
 roll_on=true;
  
 tgt_pitch-=thr_angle;
 
 if(pitch_on||yaw_on||roll_on)att_guide(calc_step,tgt_pitch,tgt_yaw,tgt_roll,use_grp);
}
//#######################################################################################//
double ap_transorbit::a (double t){return a0/(1-t/tau);}
double ap_transorbit::b0(double t){return -isp*slog(1-t/tau);}
double ap_transorbit::bn(double t,int n){if(n==0) return b0(t);return bn(t,n-1)*tau-isp*pow(t,n)/n;}
double ap_transorbit::c0(double t){return b0(t)*t-bn(t,1);}
double ap_transorbit::cn(double t,int n){if(n==0) return c0(t);return cn(t,n-1)*tau-isp*pow(t,n+1)/(n*(n+1));}
//#######################################################################################//
void ap_transorbit::step(double simt,double simdt)
{
 if(is_running){   
  if(!is_loaded)init(simt);   
  double met=simt-t0;
  if((met>(last_calc+calc_step)) || no_guidance_computed){
   no_guidance_computed=false;   
   navigate();    
   if(thrust>=1){  
    estimate(met);    
    guide(met);   
   }
   last_calc=last_calc+calc_step;
  } 
  if(stop_met<met){stop();return;}  
  
  if(use_target)            att_control_exp(met-last_calc,get_pitch_horizon(use_grp),get_heading(use_grp)   ,get_bank(use_grp),target_pitch,tgt_az    ,target_roll,pitch_on*1+yaw_on*2+roll_on*4,use_grp);
           else if(tgt_az<0)att_control_exp(met-last_calc,get_pitch_horizon(use_grp),get_slip_angle(use_grp),get_bank(use_grp),target_pitch,target_yaw,target_roll,pitch_on*1+yaw_on*2+roll_on*4,use_grp);
           else             att_control_exp(met-last_calc,get_pitch_horizon(use_grp),get_heading(use_grp)   ,get_bank(use_grp),target_pitch,tgt_az    ,target_roll,pitch_on*1+yaw_on*2+roll_on*4,use_grp);  
 }else if(is_started)init(simt);  
 c++;     //FIXME: WTF? Crashes some apps without that, FP exception.
}
//#######################################################################################//
iautopilot *mk_transorbit(SAL *sal_in,VESSEL *v){return new ap_transorbit(sal_in,v);}
//#######################################################################################//