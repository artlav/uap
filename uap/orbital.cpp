//#######################################################################################//
//UAP module, Made by Artlav in 2011
//Orbital maneuvring autopilots
//#######################################################################################//
#define _CRT_SECURE_NO_DEPRECATE 1
//#######################################################################################//
#include "sal.h"
#include "iap.h"
#include "orbital.h"
//#######################################################################################//
ap_align::ap_align(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Align planes\0");
 add_var("engine",VT_INT,&use_grp,0,1,15);
 add_var("target",VT_CHAR,tgt_nam,0,0,255);
 add_var("rinc_delta",VT_DBL,&rinc_delta,VF_DEG,0,90*RAD);
 add_var("rinc",VT_DBL,&rinc,VF_OUT|VF_DEG,-360*RAD,360*RAD);
 add_var("time_to_plane",VT_DBL,&time_to_plane,VF_OUT,0,1e10);
 add_var("burn_time",VT_DBL,&burn_time,VF_OUT,0,1e10);
 
 sprintf(tgt_nam,"ISS\0");
 use_grp=xTHGROUP_MAIN;
 rinc_delta=0.1*RAD;
 rinc=0;
 time_to_plane=0;
 burn_time=0;
 max_time_accel=10;
}
//#######################################################################################//
void ap_align::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 init();
}
//#######################################################################################//
void ap_align::init()
{
 is_loaded=true;
 
 stop_all_engines();
 
 tgt_obj=sal->object_by_name(tgt_nam);
 if(!tgt_obj){stop();return;}
}
//#######################################################################################//
void ap_align::stop()
{
 iautopilot::stop();
 stop_all_engines();
}
//#######################################################################################//
void ap_align::step(double simt,double simdt)
{
 if(!is_loaded)init();
 if(is_running){
  VECTOR3 pyr,d,r,vv,vp,pp,pv,tv,tp,vn,tn,rv,rp,dv,pd;
  double dt,bt,acc,par;
  int btw=0;
  
  //ns=set_ns;
  
  pp=sal->get_global_obj_pos(sal->get_gravity_ref(us));
  pv=sal->get_global_obj_vel(sal->get_gravity_ref(us));  
  vp=sal->get_global_pos(us);
  vv=sal->get_global_vel(us);
  tp=sal->get_global_obj_pos(tgt_obj);
  tv=sal->get_global_obj_vel(tgt_obj);
  
  vn=vmulv(nrvec(vv-pv),nrvec(vp-pp));
  tn=vmulv(nrvec(tv-pv),nrvec(tp-pp));
  
  rv=vv-pv;
  rp=vp-pp;
  
  dv=tn*(smulv(rv,tn)/modvs(tn));
  pd=tn*(smulv(rp,tn)/modvs(tn));
  
  dt=modv(pd)/modv(dv);
  rinc=sacos(smulv(nrvec(vn),nrvec(tn)));
  if(rinc<rinc_delta){stop();return;}
  
  par=smulv(nrvec(vn),nrvec(tn));
  if(par<0.5)par=0.5;
  acc=(modv(get_group_thrust_vector(use_grp,true))/sal->get_ves_mass(us))*par;
  bt=1.1*modv(dv)/acc;
  if(bt<1){
   acc=acc/10;
   bt=bt*10;
   btw=1;
  }else if(dt<10)bt=bt+10;
  
  time_to_plane=dt;
  burn_time=bt;
  
  r=sal->global_2_local(us,nrvec(vp-pp)+vp);
  
  if(smulv(rv,tn)<0){
   //Normal+
   d=sal->global_2_local(us,vn+vp);
   //sprintf(oapiDebugString(),"+, dv=%f pd=%f dt=%f bt=%f acc=%f",modv(dv),modv(pd),dt,bt,acc);
  }else{
   //Normal-
   d=sal->global_2_local(us,-vn+vp);
   //sprintf(oapiDebugString(),"-, dv=%f pd=%f dt=%f bt=%f acc=%f",modv(dv),modv(pd),dt,bt,acc);
  }
  pyr=get_pitchyawroll(d,r);
  
  if((dt<0.5*bt+60)&&(smulv(dv,pd)<0)){
   sprintf(state_string,"Full align");
   max_time_accel=10;
   if((btw)&&(dt<5))max_time_accel=1;
   att_guide(simdt,0,0,0,use_grp);
   att_control_exp(simdt,pyr.x,pyr.y,pyr.z,0,0,0,7,use_grp);
   
   if((absd(pyr.x-0)<10*RAD)&&(absd(pyr.y-0)<10*RAD))if(dt<0.5*bt+1){
    if(btw){
     sprintf(state_string,"Slow align");
     set_thgrp_level(0.1,use_grp);
    }else set_thgrp_level(1,use_grp);
   }
  }else{
   sprintf(state_string,"Waiting for node");
   stop_all_engines();
   max_time_accel=max_time_accel*2;
   if(max_time_accel>10000)max_time_accel=10000;
   if(dt<1000)max_time_accel=100;
  }
 }
}
//#######################################################################################//
//#######################################################################################//
//#######################################################################################//
ap_approach::ap_approach(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Approach\0");
 add_var("engine",VT_INT,&use_grp,0,1,15);
 add_var("target",VT_CHAR,tgt_nam,0,0,255);
 add_var("max_velocity",VT_DBL,&max_velocity,0,0,1e10);
 add_var("tgt_distance",VT_DBL,&tgt_distance,0,0,1e10);
 add_var("cur_distance",VT_DBL,&cur_distance,VF_OUT,0,1e10);
 
 sprintf(tgt_nam,"ISS\0");
 use_grp=xTHGROUP_MAIN;
 max_velocity=50;
 tgt_distance=5000;
 max_time_accel=100;
 att_split=1;
}
//#######################################################################################//
void ap_approach::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 init();
}
//#######################################################################################//
void ap_approach::init()
{
 is_loaded=true;
 corr=true;
 
 stop_all_engines();
 
 tgt_ves=sal->vessel_by_name(tgt_nam);
 if(!tgt_ves){fail_error("No such target vessel");return;}
}
//#######################################################################################//
void ap_approach::stop()
{
 iautopilot::stop();
 stop_all_engines();
}
//#######################################################################################//
void ap_approach::step(double simt,double simdt)
{
 if(!is_loaded)init();
 if(is_running){
  VECTOR3 pyr,d,vv,vp,tv,tp,rv,rp,dv;
   
  vp=sal->get_global_pos(us);
  vv=sal->get_global_vel(us);
  tp=sal->get_global_pos(tgt_ves);
  tv=sal->get_global_vel(tgt_ves);
  
  rv=vv-tv;
  rp=vp-tp;
  
  cur_distance=modv(rp); 
  
  if((modv(rp)<tgt_distance)&&(modv(rv)<1)){stop();return;}

  dv=-rv-nrvec(rp)*max_velocity;
  if(modv(rp)<tgt_distance)dv=-rv;
  d=sal->global_2_local(us,dv+vp);
  
  pyr=get_pitchyawroll(d,tvec(0,0,0));
  
  stop_all_engines();
  if(modv(d)>1)corr=true;
  if(modv(d)>5){
   max_time_accel=10;
   att_guide(simdt,0,0,0,use_grp);
   att_control_exp(simdt,pyr.x,pyr.y,0,0,0,0,7,use_grp);
  
   set_thgrp_level(0,use_grp);
   pyr.z=0;
   if((absd(pyr.x-0)<8*RAD)&&(absd(pyr.y-0)<8*RAD))set_thgrp_level(1-modv(pyr),use_grp);
  }else{
   max_time_accel=100;
   if(corr&&(modv(d)<0.1)){corr=false;return;}
   if(!corr)return;
   give_thgrp_accel(0.1,d,xTHGROUP_ATT_ALL_LIN);
  }
 }
}
//#######################################################################################//
//#######################################################################################//
//#######################################################################################//
ap_sync_orbit::ap_sync_orbit(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Sync orbits\0");
 add_var("engine",VT_INT,&use_grp,0,1,15);
 add_var("target",VT_CHAR,tgt_nam,0,0,255);
 add_var("max_dv_to_use",VT_DBL,&max_dv_to_use,0,0,1e10);
 add_var("tgt_distance",VT_DBL,&tgt_distance,0,0,1e10);
 add_var("minimize",VT_INT,&minimize,0,0,2);
 add_var("stage",VT_INT,&stage,VF_STATE,0,10);
 add_var("minimizing_by",VT_CHAR,&minimizing_by,VF_OUT,0,255);
 add_var("time_to_sync",VT_DBL,&dt,VF_OUT,0,1e10);
 add_var("cur_distance",VT_DBL,&cur_distance,VF_OUT,0,1e10);
 
 sprintf(tgt_nam,"ISS\0");
 minimize=1;
 use_grp=xTHGROUP_MAIN;
 tgt_distance=50000;
 max_dv_to_use=200;
 max_time_accel=1;
 
 reset();
}
//#######################################################################################//
void ap_sync_orbit::reset()
{
 iautopilot::reset();
 stage=0;
 tgt_ves=0;
}
//#######################################################################################//
void ap_sync_orbit::info()
{
 switch(minimize){
  case 0:set_char_var("minimizing_by","Delta-V");break;
  case 1:set_char_var("minimizing_by","Time");break;
  case 2:set_char_var("minimizing_by","Precision");break;
  default:set_char_var("minimizing_by","WTF?");
 }
}
//#######################################################################################//
void ap_sync_orbit::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 stage=0;
 init();
}
//#######################################################################################//
void ap_sync_orbit::init()
{
 is_loaded=true;
 
 stop_all_engines();
 
 tgt_ves=sal->vessel_by_name(tgt_nam);
 if(!tgt_ves){stop();return;}
}
//#######################################################################################//
void ap_sync_orbit::stop()
{
 iautopilot::stop();
 stop_all_engines();
}
//#######################################################################################//
//#######################################################################################//
//#######################################################################################//
typedef struct{
 double m,sma,ecc,inc,lan,lnp,mnl,tra,mna,eca,t,pet;
}elems_typ;
//#######################################################################################//
double get_mna_time(elems_typ el,double dt)
{
 double meanMotion,deltamna,mna;

 meanMotion=sqsrt((el.m*GGRAV)/pow(fabs(el.sma),3.0));
 deltamna=dt*meanMotion;
 //mna=fmod(el.mna+deltamna,2*PI);
 mna=fmod(el.mnl-el.lnp+deltamna,2*PI);
 if(mna<0)mna+=2*PI;
	
 return mna;
}
//#######################################################################################//
double get_eca(double mna,double ecc)
{
 int i,maxIterations;
 double relativeError,mnaEstimate,e,eccentricAnomalyEstimate,maxRelativeError;
 eccentricAnomalyEstimate=mna;
 
 maxRelativeError=1e-10;
 maxIterations=100;

 if(ecc==0)return mna;

 if(ecc==1.0)e=ecc+1e-6;else e=ecc;

 i=0;
 do{
  if(ecc<1.0){
   eccentricAnomalyEstimate=eccentricAnomalyEstimate-(eccentricAnomalyEstimate-e*sin(eccentricAnomalyEstimate)-mna)/(1-e*cos(eccentricAnomalyEstimate));
   mnaEstimate=eccentricAnomalyEstimate-e*sin(eccentricAnomalyEstimate);
  }else{
   if (eccentricAnomalyEstimate > PI) eccentricAnomalyEstimate-=2*PI;
   eccentricAnomalyEstimate=eccentricAnomalyEstimate-(e*sinh(eccentricAnomalyEstimate)-eccentricAnomalyEstimate-mna)/(e*cosh(eccentricAnomalyEstimate)-1);
   mnaEstimate=e*sinh(eccentricAnomalyEstimate)-eccentricAnomalyEstimate;
  }
  relativeError=1-mnaEstimate/mna;
  i++;
 }while((i<maxIterations)&&(fabs(relativeError)>fabs(maxRelativeError)));

 eccentricAnomalyEstimate=fmod(eccentricAnomalyEstimate,2*PI);
 if(eccentricAnomalyEstimate<0)eccentricAnomalyEstimate+=2*PI;
 
 return eccentricAnomalyEstimate;
}
//#######################################################################################//
double get_tra(double m,double ecc,double eca)
{
 double tra;
 if(ecc<1)tra=(2*satan(sqsrt((1+ecc)/(1-ecc))*tan(eca/2)));
     else tra=(sacos((cosh(eca)-ecc)/(1-ecc*cosh(eca))));
 return tra;
}
//#######################################################################################//
void elems2sv(elems_typ el,VECTOR3 &pos,VECTOR3 &vel)
{
 VECTOR3 n,h,north,vPro,vO;
 double argPos,rPe,vPe,v2,e;

 e=el.ecc;
 if(e==1.0)e+=1e-6;

 n=tvec(cos(el.lan),sin(el.lan),0.0);
 north=tvec(0.0,0.0,1.0);

 h=vmulv(n,north)*sin(el.inc);
 h.z=cos(el.inc);
 h=nrvec(h);

 if(e<1.0){
  rPe=el.sma*(1.0-sqr(e))/(1.0+e);
  vPe=sqsrt(el.m*GGRAV*(2/rPe-1/el.sma));
 }else{
  rPe=fabs(el.sma)*(sqr(e)-1.0)/(1.0+e);
  vPe=sqsrt(el.m*GGRAV*(2/rPe+1/fabs(el.sma)));
 }
 
 h=h*rPe*vPe;
 argPos=el.lnp-el.lan+el.tra;
 
 pos=vmulv(nrvec(h),n)*sin(argPos)+n*cos(argPos);

 if(e<1.0)pos=pos*(el.sma*(1.0-sqr(e))/(1.0+e*cos(el.tra)));
     else pos=pos*(fabs(el.sma)*(sqr(e)-1.0)/(1.0+e*cos(el.tra)));

 if(e<1.0)v2=el.m*GGRAV*(2/modv(pos)-1/el.sma);
     else v2=el.m*GGRAV*(2/modv(pos)+1/fabs(el.sma));

 vPro=nrvec(vmulv(h,pos))*(modv(h)/modv(pos));
 vO=nrvec(pos)*(sqsrt(v2-modvs(vPro))*sin(el.tra)/fabs(sin(el.tra)));

 vel=vPro+vO;
}
//#######################################################################################//
double acosh(double x){return slog(x+sqsrt(x*x-1));}
//#######################################################################################//
void sv2elems(elems_typ &el,VECTOR3 pos,VECTOR3 vel)
{
 VECTOR3 h,n,e;
 double absh,absn,absr,abse,E;
 int isEquatorial,isCircular,isHyperbola;
 double agp,tmp;

 h=vmulv(pos,vel);
 n=tvec(-h.y,h.x,0.0);

 absh=modv(h);
 absn=modv(n);
 absr=modv(pos);

 e=vmulv(vel,h);
 e=e*(absr/(el.m*GGRAV));
 e=e-pos;
 e=e*(1.0/absr);

 abse=modv(e);
 if(abse==1.0)abse+=1e-6;

 isEquatorial=absn<1e-6;
 isCircular  =abse<1e-6;
 isHyperbola =abse>=1.0;

 E=0.5*modvs(vel)-(el.m*GGRAV)/absr;
 if(E==0.0)E+=1e-6;

 //SMa
 el.sma=-(el.m*GGRAV)/(2.0*E);

 //Ecc
 el.ecc=abse;

 //Inc
 el.inc=sacos(h.z/absh);

 //LAN
 if(!isEquatorial){
  el.lan=sacos(n.x/absn);
  if(n.y<0.0)el.lan=2*PI-el.lan;
 }else el.lan=0;

 //AGP
 if(isCircular)agp=0;
 else if(isEquatorial){
  agp=sacos(e.x/abse);
  if(e.z<0)agp=2*PI-agp;
 }else{
  agp=sacos(smulv(n,e)/(absn*abse));
  if(e.z<0)agp=2*PI-agp;
 } 
 
 //LNP
 el.lnp=fmod(el.lan+agp,2*PI);
 
 //TrA
 if(isCircular){
  if(isEquatorial){
   el.tra=sacos(pos.x/absr);
   if(vel.x>0)el.tra=2*PI-el.tra;
  }else{
   el.tra=sacos(smulv(n,pos)/(absn*absr));
   if(smulv(n,vel)>0)el.tra=2*PI-el.tra;
  }
 }else{
  tmp=smulv(e,pos)/(abse*absr);

  if(tmp<=-1.0)el.tra=PI;
  else if(tmp>=1.0)el.tra=0.0;
  else el.tra=sacos(tmp);

  if(smulv(pos,vel)<0)el.tra=2*PI-el.tra;
 }
 
 
      if(isHyperbola)el.eca=acosh((1.0-absr/el.sma)/el.ecc);
 else if(isCircular)el.eca=0.0;
 else{ 
  tmp=(1.0-absr/el.sma)/el.ecc;
  if(tmp<=-1.0)el.eca=PI;
  else if(tmp>=1.0)el.eca=0.0;
  else el.eca=sacos(tmp);
 }
 
 if(isHyperbola){
  if(sin(el.tra)*el.eca<0)el.eca=-el.eca; 
 }else{ 
  if(sin(el.tra)<0)el.eca=2*PI-el.eca;
 }

 if(isHyperbola)el.mna=el.ecc*sinh(el.eca)-el.eca; 
           else el.mna=el.eca-el.ecc*sin(el.eca);

 //MNL
 el.mnl=fmod(el.mna+el.lnp,2*PI); 
 
 //T
 el.t=2*PI*sqsrt(fabs(cub(el.sma)/(el.m*GGRAV))); 
 
 double tPe=el.mna*el.t/(2*PI);

 if(isHyperbola)el.pet=-tPe;
 else el.pet=el.t-tPe;
}
//#######################################################################################//
double dist(elems_typ ve,elems_typ te,double n)
{
 VECTOR3 nv,npv,npt;
 ve.tra=get_tra(ve.m,ve.ecc,get_eca(get_mna_time(ve,n),ve.ecc));
 te.tra=get_tra(te.m,te.ecc,get_eca(get_mna_time(te,n),te.ecc));
 elems2sv(ve,npv,nv);
 elems2sv(te,npt,nv);
 return modv(npv-npt);
}
//#######################################################################################//
double test_isecs(double &dt,VECTOR3 vp,VECTOR3 vv,VECTOR3 tp,VECTOR3 tv,double m)
{
 elems_typ ve,te;
 int icf;
 double t,step,dx,dr,k,n=0,a,b,c,db1,db,mdst;
 
 step=0.1;
 
 ve.m=m;
 te.m=m;
 sv2elems(ve,vp,vv);
 sv2elems(te,tp,tv); 
 t=2*PI*sqsrt(fabs(ve.sma*ve.sma*ve.sma/(GGRAV*m)));

 dt=0;
 mdst=1e10; 
 icf=0;
 
 dr=dist(ve,te,-t*step);
 for(n=0;n<5*t;n+=t*step){
  dx=dist(ve,te,n);
  if(dx>dr){
   if(icf){dr=dx;continue;}
   icf=1;
   
   k=t*step;
   a=n-2*k;
   b=n-1*k;
   c=n;
   db1=dist(ve,te,b-1);
   if(db1<dr)c=n-1*k;
        else a=n-1*k;
   
   while(k>1){
    k=k/2;
    b=a+(c-a)/2;
    db=dist(ve,te,b);
    db1=dist(ve,te,b-1);
    if(db1<db)c=b;
         else a=b;
   }
   if(db<mdst){dt=b;mdst=db;}
  }else icf=0;
  dr=dx;
 }
 return mdst;
}
//#######################################################################################//
void ap_sync_orbit::step(double simt,double simdt)
{
 if(!is_loaded)init();
 if(is_running){
  if(sal->get_gravity_ref(tgt_ves)!=sal->get_gravity_ref(us)){stop();return;}
  
  VECTOR3 vp,vv,tp,tv,rv,rp,ang_vel,pyr;
  OBJHANDLE ref;
  double m,acc,lv,d;
  int i,s;
  
  ref=sal->get_gravity_ref(us);
  m=sal->get_obj_mass(ref);
  vp=sal->get_relative_pos(us,ref);
  vv=sal->get_relative_vel(us,ref);
  tp=sal->get_relative_pos(tgt_ves,ref);
  tv=sal->get_relative_vel(tgt_ves,ref);
  ang_vel=get_angular_vel_grp(use_grp);
  
  rv=vv-tv;
  rp=vp-tp;
  cur_distance=modv(rp); 
  pyr=get_pitchyawroll(sal->global_2_local(us,vv+sal->get_global_pos(us)),sal->global_2_local(us,vp+sal->get_global_pos(us)));
 
  if(cur_distance<tgt_distance){stop();return;}
  
  if(stage==0){
   mindp=1e10;
   was=0;
   ws_cnt=0;
   ws_cur=0;
   stage=1;
  }
  
  if(stage==1){
   sprintf(state_string,"Thinking...\0");
   max_time_accel=1;
   s=0;
   for(i=ws_cur;i<max_dv_to_use*2;i+=1){
    dv=i/2.0;
    ws_cur=i;
    s++;
    if(s>50)return;
    
    d=test_isecs(dt,vp,vv+nrvec(vv)*dv,tp,tv,m);
    if(d<tgt_distance)if(d<mindp){
     mindp=d;
     adv[ws_cnt]=dv;
     adp[ws_cnt]=d;
     adt[ws_cnt]=dt;
     was=1;
     continue;
    }
    if(was)ws_cnt++;
    if(ws_cnt>=100)break;
    was=0;
    mindp=tgt_distance;
   }
   if(was)ws_cnt++;
   if(ws_cnt==0){fail_error("Cannot find sync solution");return;}
   
   //FILE* ouf=fopen("sync_orbits.csv","a");
   //fprintf(ouf,"\ncur_distance=%f\n",cur_distance); 
   //fprintf(ouf,"dv \t\tdt \t\tdst\n"); 
   //for(i=0;i<ws_cnt;i++)fprintf(ouf,"%f \t%f \t%f\n",adv[i],adt[i],adp[i]);
   
   s=0;
   if(ws_cnt!=1){
    mindp=1e50;
    switch(minimize){
     case 0:{
      s=-1;
      for(i=0;i<ws_cnt;i++)if(adp[i]<tgt_distance/3){s=i;break;}
      if(s==-1)s=0;
      break;
     }
     case 1:{
      s=-1;
      for(i=0;i<ws_cnt;i++)if(adp[i]<tgt_distance/3)if(adt[i]<mindp){s=i;mindp=adt[i];}
      if(s==-1)for(i=0;i<ws_cnt;i++)if(adt[i]<mindp){s=i;mindp=adt[i];}
      break;
     }
     case 2:{
      for(i=0;i<ws_cnt;i++)if(adp[i]<mindp){s=i;mindp=adp[i];}
      break;
     }
    }
   }
   
   //fprintf(ouf,"select=%d (%f\t%f\t%f)\n",s,adv[s],adt[s],adp[s]);
   //fclose(ouf);
   
   mindp=adp[s];
   dt=adt[s];
   dv=adv[s];
   stage=2;
  }
  if(stage==2){
   sprintf(state_string,"Aligning\0");
   max_time_accel=10;
   if((absd(pyr.x-0)<1*RAD)&&(absd(pyr.y-0)<1*RAD)&&(absd(pyr.z-0)<1*RAD)){
    if((absd(ang_vel.x)<0.05)&&(absd(ang_vel.y)<0.05)&&(absd(ang_vel.z)<0.05)){stage=3;return;}
   }
   att_guide(simdt,0,0,0,use_grp);
   att_control_exp(simdt,pyr.x,pyr.y,pyr.z,0,0,0,7,use_grp);
  }
  if(stage==3){
   sprintf(state_string,"Firing\0");
   if(dv<=0){stop_all_engines();tt=dt;stage=4;rt=simt;return;}
   max_time_accel=1;
   att_guide(simdt,0,0,0,use_grp);
   att_control_exp(simdt,pyr.x,pyr.y,pyr.z,0,0,0,7,use_grp);
   
   acc=modv(get_group_thrust_vector(use_grp,true))/sal->get_ves_mass(us);
   if(acc*simdt>dv)lv=dv/simdt;else lv=acc;
   
   dv=dv-lv*simdt;
   set_thgrp_level(lv/acc,use_grp);
  }
  if(stage==4){
   sprintf(state_string,"Waiting\0");
   max_time_accel=max_time_accel*2;
   if(max_time_accel>10000)max_time_accel=10000;
   dt=tt-(simt-rt);
   if(dt<0)stage=0;
   set_thgrp_level(0,use_grp);
  }
 }
}
//#######################################################################################//
//#######################################################################################//
//#######################################################################################//
ap_hohmann::ap_hohmann(SAL *sal_in,VESSEL *vessel):iautopilot(sal_in,vessel)
{
 sprintf(name,"Hohmann central\0");
 add_var("engine",VT_INT,&use_grp,0,1,15);
 add_var("target",VT_CHAR,tgt_nam,0,0,255);
 add_var("tgt_orbit_alt",VT_DBL,&tgt_orbit_alt,0,0,1e10);
 add_var("stage",VT_INT,&stage,VF_STATE,0,1e10);
 add_var("time_to_burn",VT_DBL,&dt,VF_OUT,0,1e10);
 add_var("delta-v",VT_DBL,&dv,VF_OUT,0,1e10);
 add_var("target_influence",VT_DBL,&g_tgt,VF_OUT,0,1e10);
 
 sprintf(tgt_nam,"Moon\0");
 use_grp=xTHGROUP_MAIN;
 tgt_orbit_alt=50000;
 stage=0;
 dv=0;
 dt=0;
 dir=0;
 g_tgt=0;
}
//#######################################################################################//
void ap_hohmann::start()
{
 iautopilot::start();
 is_started=true;
 is_running=true;
 init();
}
//#######################################################################################//
double angle_between_vectors(VECTOR3 a,VECTOR3 b,VECTOR3 up)
{
 double ang;
 VECTOR3 axis;
 
 ang=sacos(smulv(nrvec(a),nrvec(b)));
 axis=nrvec(vmulv(a,b));
 if(smulv(axis,up)<0)ang=2*PI-ang;
 
 return 2*PI-ang;
}
//#######################################################################################//
void ap_hohmann::init()
{
 is_loaded=true;
 
 stop_all_engines();
 
 tgt_obj=sal->object_by_name(tgt_nam);
 if(!tgt_obj){stop();return;}

 //Fix for -;
 if(stage<2){
  VECTOR3 vv,vp,tv,tp,pv,pp,vrv,vrp,trv,trp;
  double mu,m,t,t0,mna,tx,sz;
  elems_typ ve,te;
  int n;
   
  vp=sal->get_global_pos(us);
  vv=sal->get_global_vel(us);
  tp=sal->get_global_obj_pos(tgt_obj);
  tv=sal->get_global_obj_vel(tgt_obj);
  pp=sal->get_global_obj_pos(sal->get_gravity_ref(us));
  pv=sal->get_global_obj_vel(sal->get_gravity_ref(us));
  
  sz=sal->get_obj_size(tgt_obj);
  m=sal->get_obj_mass(sal->get_gravity_ref(us))+sal->get_obj_mass(tgt_obj);
  mu=GGRAV*m;
  te.m=m;
  ve.m=m;
  
  vrv=vv-pv;
  vrp=vp-pp;
  trv=tv-pv;
  trp=tp-pp;
   
  t=PI*sqsrt(cub(modv(vrp)+modv(trp)+sz)/(8*mu));
  t0=t;
  
  sv2elems(te,trp,trv); 
  sv2elems(ve,vrp,vrv); 
  
  n=0;tx=1;
  while(n<100){ 
   te.tra=get_tra(m,te.ecc,get_eca(get_mna_time(te,t),te.ecc));
   elems2sv(te,trp,trv);
   ttp=-trp;
   
   mna=ve.mna;
   if(tx>0)ve.tra+=angle_between_vectors(vrp,ttp,nrvec(vmulv(vrv,vrp)));
      else ve.tra-=2*PI-angle_between_vectors(vrp,ttp,nrvec(vmulv(vrv,vrp)));
   elems2sv(ve,vrp,vrv);
   sv2elems(ve,vrp,vrv);
   
   if(tx>0){
    if(ve.mna<mna)ve.mna+=2*PI;
    dt=((ve.mna-mna)/(2*PI))*ve.t;
   }else{
    //WTF?
    if(ve.mna<mna)ve.mna-=2*PI;
    dt=-((mna-ve.mna)/(2*PI))*ve.t;
   }
   
   t=PI*sqsrt(cub(modv(vrp)+modv(trp)+sz)/(8*mu))+dt;
   if(fabs(dt)<1)break;
   tx=dt;
   n++;
  }
  
  tgt_apd=modv(ttp)+sz;
  dt=t-t0;
  dv=sqsrt(mu/modv(vrp))*(sqsrt(2*modv(ttp)/(modv(vrp)+modv(ttp)))-1);
 }
}
//#######################################################################################//
void ap_hohmann::stop()
{
 iautopilot::stop();
 stop_all_engines();
}
//#######################################################################################//
void ap_hohmann::step(double simt,double simdt)
{
 if(!is_loaded)init();
 if(is_running){
  VECTOR3 pyr,vrv,vrp,vp,ang_vel;
  double mna,acc,bt,apd,pea,ped;
  elems_typ ve;
 
  vp=sal->get_global_pos(us);
  ang_vel=get_angular_vel_grp(use_grp);
  vrp=sal->get_relative_pos(us,sal->get_gravity_ref(us));
  vrv=sal->get_relative_vel(us,sal->get_gravity_ref(us));
  ve.m=sal->get_obj_mass(sal->get_gravity_ref(us));
  sv2elems(ve,vrp,vrv); 
  
  pyr=get_pitchyawroll(sal->global_2_local(us,sal->get_relative_vel(us,sal->get_gravity_ref(us))+sal->get_global_pos(us)),sal->global_2_local(us,sal->get_relative_pos(us,sal->get_gravity_ref(us))+sal->get_global_pos(us)));
  acc=modv(get_group_thrust_vector(use_grp,true))/sal->get_ves_mass(us);
  bt=dv/acc;
  
  max_time_accel=max_time_accel*2;
  
  if(stage==0){
   sprintf(state_string,"Waiting for ejection");
   mna=ve.mna;
   ve.tra+=angle_between_vectors(vrp,ttp,nrvec(vmulv(vrv,vrp)));
   elems2sv(ve,vrp,vrv);
   sv2elems(ve,vrp,vrv);
   if(ve.mna<mna)ve.mna+=2*PI;
   dt=((ve.mna-mna)/(2*PI))*ve.t;
   
   if(dt-bt/2<0){
    stage=1;
    max_time_accel=10;
   }else if(dt-bt/2<60){
    sprintf(state_string,"Aligning");
    max_time_accel=10;
    att_guide(simdt,0,0,0,use_grp);
    att_control_exp(simdt,pyr.x,pyr.y,0,0,0,0,7,use_grp);
   }else{
    if(max_time_accel>10000)max_time_accel=10000;
    if(dt-bt/2<1000)max_time_accel=100;
    stop_all_engines();
   }
   return;
  }
  if(stage==1){
   sprintf(state_string,"Ejection burn");
   if(max_time_accel>10)max_time_accel=10;
   att_guide(simdt,0,0,0,use_grp);
   att_control_exp(simdt,pyr.x,pyr.y,0,0,0,0,7,use_grp);
   
   apd=ve.sma*(1.0+ve.ecc);
   if(apd>=tgt_apd){stop_all_engines();stage=2;return;}
   if(tgt_apd-apd<0.3*(tgt_apd-(ve.sma*(1.0-ve.ecc)))){max_time_accel=1;}
   set_thgrp_level(1,use_grp);
   return;
  }
  
  dt=0;
  dv=0;
  
  vrp=sal->get_relative_pos(us,tgt_obj);
  vrv=sal->get_relative_vel(us,tgt_obj);
  ve.m=sal->get_obj_mass(tgt_obj);
  sv2elems(ve,vrp,vrv); 
  pea=ve.sma*(1.0-ve.ecc)-sal->get_obj_size(tgt_obj);
  
  
  double Gref=sal->get_obj_mass(tgt_obj)/modvs(vrp);
  double Gtot=0;
  int numcb=sal->get_cb_count();
  for(int i=0;i<numcb;i++){
   OBJHANDLE obj=sal->cb_by_index(i);
   VECTOR3 pos=sal->get_relative_pos(us,obj);
   double Gobj=sal->get_obj_mass(obj)/modvs(pos);
   Gtot+=Gobj;
  }
  g_tgt=Gref/Gtot;
   
  if(stage==2){
   if(max_time_accel>10000)max_time_accel=10000;
   sprintf(state_string,"Waiting for encounter");
   
   if((g_tgt>0.6)||(modv(vrp)<4*sal->get_obj_size(tgt_obj))){stage=3;return;}
   if(g_tgt>0.58){max_time_accel=100;}
   return;
  } 
  if(stage==3){
   sprintf(state_string,"Correction");
   if(max_time_accel>10)max_time_accel=10;
   if(pea<tgt_orbit_alt){
    pyr=get_pitchyawroll(sal->global_2_local(us,nrvec(vmulv(vmulv(vrv,vrp),vrv))+vp),tvec(0,0,0));
    if(dir==0)dir=1;
   }else if(pea>tgt_orbit_alt){
    pyr=get_pitchyawroll(-sal->global_2_local(us,nrvec(vmulv(vmulv(vrv,vrp),vrv))+vp),tvec(0,0,0));
    if(dir==0)dir=-1;
   }
   
   if((pea<tgt_orbit_alt)&&(dir==-1)){stop_all_engines();stage=4;return;}
   if((pea>tgt_orbit_alt)&&(dir== 1)){stop_all_engines();stage=4;return;}
   
   att_guide(simdt,0,0,0,use_grp);
   att_control_exp(simdt,pyr.x,pyr.y,0,0,0,0,7,use_grp);
   
   set_thgrp_level(0,use_grp);
   pyr.z=0;
   if((absd(pyr.x-0)<5*RAD)&&(absd(pyr.y-0)<5*RAD)){
    if((absd(ang_vel.x)<0.10)&&(absd(ang_vel.y)<0.10)){max_time_accel=1;set_thgrp_level(1-modv(pyr),use_grp);}
   }
   return;
  }
  if(stage==4){
   sprintf(state_string,"Waiting for closser encounter");
   if(max_time_accel>10000)max_time_accel=10000;
   dir=0;
   if((g_tgt>0.9)||(modv(vrp)<2*sal->get_obj_size(tgt_obj))){stage=5;return;}
   if(g_tgt>0.88){max_time_accel=100;}
   return;
  } 
  if(stage==5){
   sprintf(state_string,"Second correction");
   if(max_time_accel>10)max_time_accel=10;
   if(pea<tgt_orbit_alt){
    pyr=get_pitchyawroll(sal->global_2_local(us,nrvec(vmulv(vmulv(vrv,vrp),vrv))+vp),tvec(0,0,0));
    if(dir==0)dir=1;
   }else if(pea>tgt_orbit_alt){
    pyr=get_pitchyawroll(-sal->global_2_local(us,nrvec(vmulv(vmulv(vrv,vrp),vrv))+vp),tvec(0,0,0));
    if(dir==0)dir=-1;
   }
   
   if((pea<tgt_orbit_alt)&&(dir==-1)){stop_all_engines();stage=6;return;}
   if((pea>tgt_orbit_alt)&&(dir== 1)){stop_all_engines();stage=6;return;}
   
   att_guide(simdt,0,0,0,use_grp);
   att_control_exp(simdt,pyr.x,pyr.y,0,0,0,0,7,use_grp);
   
   set_thgrp_level(0,use_grp);
   pyr.z=0;
   if((absd(pyr.x-0)<5*RAD)&&(absd(pyr.y-0)<5*RAD)){
    if((absd(ang_vel.x)<0.10)&&(absd(ang_vel.y)<0.10)){max_time_accel=1;set_thgrp_level(1-modv(pyr),use_grp);}
   }
   return;
  }
  
  dt=ve.pet;
  dv=sqsrt(ve.m*GGRAV/modv(vrp));
  acc=modv(get_group_thrust_vector(use_grp,true))/sal->get_ves_mass(us);
  bt=dv/acc;
  pyr=get_pitchyawroll(-sal->global_2_local(us,vrv+vrp*(smulv(vrv,vrp)/sqr(modv(vrp)))+sal->get_global_pos(us)),tvec(0,0,0));
   
  if(stage==6){
   if(max_time_accel>10000)max_time_accel=10000;
   sprintf(state_string,"Waiting for insertion");
   
   if(dt-bt/2<0){
    stage=7;
   }else if(dt-bt/2<60){
    sprintf(state_string,"Spinning retrograde");
    max_time_accel=10;
    att_guide(simdt,0,0,0,use_grp);
    att_control_exp(simdt,pyr.x,pyr.y,0,0,0,0,7,use_grp);
   }else{
    if(max_time_accel>10000)max_time_accel=10000;
    if(dt-bt/2<1000){max_time_accel=100;}
    stop_all_engines();
   }
   return;
   
  }
  if(stage==7){
   sprintf(state_string,"Entering orbit");
   if(max_time_accel>10)max_time_accel=10;
   att_guide(simdt,0,0,0,use_grp);
   att_control_exp(simdt,pyr.x,pyr.y,0,0,0,0,7,use_grp);
   
   //if(ve.ecc>=1)apd=-1;else apd=ve.sma*(1.0+ve.ecc);
   ped=ve.sma*(1.0-ve.ecc);
   //if(((apd>0)&&(apd<=tgt_orbit_alt+get_obj_size(tgt_obj)))||(ped)){stop();return;}
   //if(ped<modv(vrp)){stop();return;}
   if((ve.ecc<1)&&(ve.sma<tgt_orbit_alt+sal->get_obj_size(tgt_obj))){stop();return;}
   set_thgrp_level(1,use_grp);
   return;
  }
  
 }
}
//#######################################################################################//
iautopilot *mk_hohmann(SAL *sal_in,VESSEL *v){return new ap_hohmann(sal_in,v);}
iautopilot *mk_sync_orbit(SAL *sal_in,VESSEL *v){return new ap_sync_orbit(sal_in,v);}
iautopilot *mk_approach(SAL *sal_in,VESSEL *v){return new ap_approach(sal_in,v);}
iautopilot *mk_align(SAL *sal_in,VESSEL *v){return new ap_align(sal_in,v);}
//#######################################################################################//




