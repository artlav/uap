//#######################################################################################//
//UAP core, Made by Artlav in 2011
//Simulator Abstraction Layer, derived functions
//#######################################################################################//
#define _CRT_SECURE_NO_WARNINGS
//#######################################################################################//
#include "sal.h"
#include "sal_imp.h"
//#######################################################################################//
#ifdef USE_ORBITER
#include "sal_orb.cpp"
#else
#ifndef WIN32
#include <stdint.h>
#endif
#include "sal_dll.cpp"
#endif        
//#######################################################################################//
MATRIX3 SAL_IMP::planet_horizon_matrix(OBJHANDLE p,VECTOR3 pos)
{
 VECTOR3 x,y,z;
 MATRIX3 rotm=get_obj_rot_matrix(p);
 y=nrvec(lvmat(rotm,pos));
 x=nrvec(vmulv(y,tvec(0,1,0)));
 z=vmulv(x,y);
 MATRIX3 hmat=tmat(x.x,x.y,x.z,y.x,y.y,y.z,z.x,z.y,z.z);
 return mulm(hmat,rotm);
}
//#######################################################################################//
double SAL_IMP::get_obj_equ_inclination(OBJHANDLE p,OBJHANDLE ref)
{
 VECTOR3 h=lvmat(get_obj_rot_matrix(ref),vmulv(get_obj_relative_pos(p,ref),get_obj_relative_vel(p,ref)));
 return sacos(-h.y/modv(h));
}
//#######################################################################################//
void SAL_IMP::get_ves_equ_pos(VESSEL *us,double &longitude,double &latitude,double &radius)
{
 MATRIX3 rotm;
 VECTOR3 equ;
 OBJHANDLE p;
 
 p=get_surface_ref(us);
 rotm=get_obj_rot_matrix(p);
 equ=vrec2sphv(lvmat(rotm,get_relative_pos(us,p)));
 longitude=equ.y;
 latitude=equ.x;
 radius=equ.z;
}
//#######################################################################################//
VECTOR3 SAL_IMP::horizon_rot   (VESSEL *us,const VECTOR3 rloc){OBJHANDLE p=get_surface_ref(us);return lvmat(planet_horizon_matrix(p,get_relative_pos(us,p)),local_2_global(us,rloc)-get_global_pos(us));}
VECTOR3 SAL_IMP::rot_horizon   (VESSEL *us,const VECTOR3 rloc){OBJHANDLE p=get_surface_ref(us);return global_2_local(us,rvmat(planet_horizon_matrix(p,get_relative_pos(us,p)),rloc)+get_global_pos(us));}
VECTOR3 SAL_IMP::global_2_local(VESSEL *us,const VECTOR3 rloc){return lvmat(get_obj_rot_matrix(get_ves_obj(us)),rloc-get_global_pos(us));}
VECTOR3 SAL_IMP::local_2_global(VESSEL *us,const VECTOR3 rloc){return rvmat(get_obj_rot_matrix(get_ves_obj(us)),rloc)+get_global_pos(us);}
//#######################################################################################//
//Courtesy LaunchMFD
//FIX: Only -90..90
double SAL_IMP::calculate_azimuth(VESSEL *ves,double tgt_radius,double tgt_inc)
{
 double azimuth,mi,vel_n,vel_e,tgt_orb_n,tgt_orb_e,lv_n,lv_e,ves_lat,ves_lon,ves_rad;
 VECTOR3 pos,vel;
 MATRIX3 rot;
 
 pos=get_relative_pos(ves,get_gravity_ref(ves));
 vel=get_relative_vel(ves,get_gravity_ref(ves));
 rot=get_obj_rot_matrix(get_gravity_ref(ves));
 
 pos=lvmat(rot,pos);
 vel=lvmat(rot,vel);
 Crt2Pol(pos,vel); 
 
 get_ves_equ_pos(ves,ves_lon,ves_lat,ves_rad);
 vel_e=vel.y*pos.x*cos(ves_lat);
 vel_n=vel.z*pos.x;
 
 double par=cos(tgt_inc)/cos(ves_lat);
 if(par>1)azimuth=PI/2+PI-sasin(par-1);
 else azimuth=sasin(par);
 
 mi=GGRAV*get_obj_mass(get_gravity_ref(ves));
 tgt_orb_n=sqsrt(mi/tgt_radius)*cos(azimuth);
 tgt_orb_e=sqsrt(mi/tgt_radius)*sin(azimuth); 
 
 lv_n=tgt_orb_n-absd(vel_n);
 lv_e=tgt_orb_e-(vel_e);

 if(lv_n==0)lv_n=0.0001;
 return satan(lv_e/lv_n);
}
//#######################################################################################//
VECTOR3 SAL_IMP::sph_2_rec(OBJHANDLE p,double vlat,double vlon,double vrad)
{
 VECTOR3 a;
 
 a.x=cos(vlat)*cos(vlon)*vrad;
 a.z=cos(vlat)*sin(vlon)*vrad;
 a.y=sin(vlat)*vrad;
 return rvmat(get_obj_rot_matrix(p),a);
}
//#######################################################################################//
VECTOR3 SAL_IMP::get_ves_airspeed_vector(VESSEL *us)
{
 VECTOR3 rp,vr;
 MATRIX3 rm;
 OBJHANDLE p;
 
 p=get_surface_ref(us);
 rm=get_obj_rot_matrix(p); 
 rp=nrvec(lvmat(rm,get_relative_pos(us,p)))*get_obj_size(p);
 vr=nrvec(rvmat(rm,vmulv(rp,tvec(0,1,0))))*(modv(tvec(rp.x,0,rp.z))*2*PI/get_planet_rotrate(p));
 return global_2_local(us,get_relative_vel(us,p)-vr+get_global_pos(us));
}
//#######################################################################################//
VECTOR3 SAL_IMP::get_ves_horizon_airspeed_vector(VESSEL *us){return horizon_rot(us,get_ves_airspeed_vector(us));} 
//#######################################################################################//
THGROUP_TYPE SAL_IMP::int2thgroup_type(int tg)
{
 switch(tg){
  case xTHGROUP_MAIN:         return THGROUP_MAIN;
  case xTHGROUP_RETRO:        return THGROUP_RETRO;
  case xTHGROUP_HOVER:        return THGROUP_HOVER;
  case xTHGROUP_MAIN_INV:     return THGROUP_MAIN;
  case xTHGROUP_ATT_PITCHUP:  return THGROUP_ATT_PITCHUP;
  case xTHGROUP_ATT_PITCHDOWN:return THGROUP_ATT_PITCHDOWN;
  case xTHGROUP_ATT_YAWLEFT:  return THGROUP_ATT_YAWLEFT;
  case xTHGROUP_ATT_YAWRIGHT: return THGROUP_ATT_YAWRIGHT;
  case xTHGROUP_ATT_BANKLEFT: return THGROUP_ATT_BANKLEFT;
  case xTHGROUP_ATT_BANKRIGHT:return THGROUP_ATT_BANKRIGHT;
  case xTHGROUP_ATT_RIGHT:    return THGROUP_ATT_RIGHT;
  case xTHGROUP_ATT_LEFT:     return THGROUP_ATT_LEFT;
  case xTHGROUP_ATT_UP:       return THGROUP_ATT_UP;
  case xTHGROUP_ATT_DOWN:     return THGROUP_ATT_DOWN;
  case xTHGROUP_ATT_FORWARD:  return THGROUP_ATT_FORWARD;
  case xTHGROUP_ATT_BACK:     return THGROUP_ATT_BACK;
  default:return THGROUP_USER;
 }
}
//#######################################################################################//
int SAL_IMP::thgroup_type2int(THGROUP_TYPE tg)
{
 switch(tg){
  case THGROUP_MAIN:         return xTHGROUP_MAIN;
  case THGROUP_RETRO:        return xTHGROUP_RETRO;
  case THGROUP_HOVER:        return xTHGROUP_HOVER;
  case THGROUP_ATT_PITCHUP:  return xTHGROUP_ATT_PITCHUP;
  case THGROUP_ATT_PITCHDOWN:return xTHGROUP_ATT_PITCHDOWN;
  case THGROUP_ATT_YAWLEFT:  return xTHGROUP_ATT_YAWLEFT;
  case THGROUP_ATT_YAWRIGHT: return xTHGROUP_ATT_YAWRIGHT;
  case THGROUP_ATT_BANKLEFT: return xTHGROUP_ATT_BANKLEFT;
  case THGROUP_ATT_BANKRIGHT:return xTHGROUP_ATT_BANKRIGHT;
  case THGROUP_ATT_RIGHT:    return xTHGROUP_ATT_RIGHT;
  case THGROUP_ATT_LEFT:     return xTHGROUP_ATT_LEFT;
  case THGROUP_ATT_UP:       return xTHGROUP_ATT_UP;
  case THGROUP_ATT_DOWN:     return xTHGROUP_ATT_DOWN;
  case THGROUP_ATT_FORWARD:  return xTHGROUP_ATT_FORWARD;
  case THGROUP_ATT_BACK:     return xTHGROUP_ATT_BACK;
  default:return 0;
 }
}
//#######################################################################################//