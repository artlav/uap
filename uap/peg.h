//#######################################################################################//
//UAP module, Made by Artlav in 2011
//Launch autopilot
//Based on OrbiterPEG by Chris Jeppesen, 2007
//#######################################################################################//
class ap_transorbit:public iautopilot{
public:
 ap_transorbit(SAL *sal_in,VESSEL *vessel);
 void navigate();
 void estimate(double ct);
 void guide(double ct);
 void target_thing(OBJHANDLE ref,OBJHANDLE tgt);
 void init(double ct);
 void start();
 void stop();
 void info();
 void reset();
 void step(double simt,double simdt);
 double a (double t);
 double b0(double t);
 double bn(double t,int n);
 double c0(double t);
 double cn(double t,int n);
 
 
 char kind_is[255],target[255];
 OBJHANDLE tgt_obj;
 bool pitch_on,yaw_on,roll_on,no_guidance_computed;
 double stop_dt,last_calc,calc_step,t0;
 int use_target,mode,use_grp,kind;

 double tgt_rad_apoapsis,tgt_rad_periapsis,tgt_ecc,tgt_periapsis,tgt_apoapsis,tgt_ta,tgt_az;

 double vr,vh,ref_dst,h,theta,omega,phi;
 VECTOR3 rh0,pos_oftgt,vel_oftgt,rmh;
 double mu,g,thrust,m,isp,a0,tau,thr_angle;

 //Estimation variables
 double rT,vrT,t_cutoff,p,deltah,deltav,stop_met;
 double ftheta,fdottheta,fdotdottheta,rbar;
 double deltatheta,thetaT;
 double fr,fdotr;
 double d1,d2,d3,d4;
 double A,B,C,last_control_upd,fhdotrh;
};
//#######################################################################################//
iautopilot *mk_transorbit(SAL *sal_in,VESSEL *v);
//#######################################################################################//
