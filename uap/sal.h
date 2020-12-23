//#######################################################################################//
//UAP core, Made by Artlav in 2011
//Simulator Abstraction Layer, definitions
//#######################################################################################//
#ifndef _sal_H
#define _sal_H  
//#######################################################################################//
//#define USE_ORBITER
//#######################################################################################//
//#include "windows.h"
#include "stdio.h"
#include <math.h>
#ifdef USE_ORBITER
#include "orbitersdk.h"
#endif
#include "util.h"  
//#######################################################################################//
#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef min
	#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif           
//#######################################################################################//
#define	xTHGROUP_MAIN          0x0001
#define	xTHGROUP_RETRO         0x0002
#define	xTHGROUP_HOVER         0x0004
#define	xTHGROUP_MAIN_INV      0x0008
#define	xTHGROUP_ATT_PITCHUP   0x0010
#define	xTHGROUP_ATT_PITCHDOWN 0x0020
#define	xTHGROUP_ATT_YAWLEFT   0x0040
#define	xTHGROUP_ATT_YAWRIGHT  0x0080
#define	xTHGROUP_ATT_BANKLEFT  0x0100
#define	xTHGROUP_ATT_BANKRIGHT 0x0200
#define	xTHGROUP_ATT_RIGHT     0x0400
#define	xTHGROUP_ATT_LEFT      0x0800
#define	xTHGROUP_ATT_UP        0x1000
#define	xTHGROUP_ATT_DOWN      0x2000
#define	xTHGROUP_ATT_FORWARD   0x4000
#define	xTHGROUP_ATT_BACK      0x8000

#define	xTHGROUP_ATT_ALL_LIN      xTHGROUP_ATT_RIGHT|xTHGROUP_ATT_LEFT|xTHGROUP_ATT_UP|xTHGROUP_ATT_DOWN|xTHGROUP_ATT_FORWARD|xTHGROUP_ATT_BACK
#define	xTHGROUP_ATT_ALL_NONMOM   xTHGROUP_ATT_ALL_LIN|xTHGROUP_MAIN|xTHGROUP_RETRO|xTHGROUP_HOVER
#define	xTHGROUP_ATT_ALL_MAJOR    xTHGROUP_MAIN|xTHGROUP_RETRO|xTHGROUP_HOVER
//#######################################################################################//
#ifndef USE_ORBITER
//#######################################################################################//
#define UINT unsigned int
#define DWORD unsigned int
//#######################################################################################//
#define	THGROUP_MAIN          0x0001
#define	THGROUP_RETRO         0x0002
#define	THGROUP_HOVER         0x0004
#define	THGROUP_MAIN_INV      0x0008
#define	THGROUP_ATT_PITCHUP   0x0010
#define	THGROUP_ATT_PITCHDOWN 0x0020
#define	THGROUP_ATT_YAWLEFT   0x0040
#define	THGROUP_ATT_YAWRIGHT  0x0080
#define	THGROUP_ATT_BANKLEFT  0x0100
#define	THGROUP_ATT_BANKRIGHT 0x0200
#define	THGROUP_ATT_RIGHT     0x0400
#define	THGROUP_ATT_LEFT      0x0800
#define	THGROUP_ATT_UP        0x1000
#define	THGROUP_ATT_DOWN      0x2000
#define	THGROUP_ATT_FORWARD   0x4000
#define	THGROUP_ATT_BACK      0x8000
#define THGROUP_USER          0x10000

#define AIRCTRL_ELEVATOR      0x0001
#define AIRCTRL_RUDDER        0x0002
#define AIRCTRL_AILERON       0x0004
#define AIRCTRL_FLAP          0x0008
#define AIRCTRL_ELEVATORTRIM  0x0010
#define AIRCTRL_RUDDERTRIM    0x0020    
//#######################################################################################//
#define OAPI_KEY_G 1
#define OAPI_KEY_K 2
#define OAPI_KEY_J 3
#define OAPI_KEY_F 4
#define OAPI_KEY_E 5
#define OAPI_KEY_O 6
//#######################################################################################//
typedef void VESSEL;
//#######################################################################################//
typedef struct{
 int tp,id;
 char *name;
 double m,r;
 VECTOR3 pos,gps,glp,vel,rot,dir,vrot;
 MATRIX3 rotm;
 void *rob;
 double rtr;
}objtype;

typedef objtype *OBJHANDLE;

typedef void *THRUSTER_HANDLE;
typedef void *DOCKHANDLE;

typedef int THGROUP_TYPE;
typedef int AIRCTRL_TYPE;
typedef int FILEHANDLE;
//#######################################################################################//
#endif
//#######################################################################################//
class SAL{
public: 
 virtual double sim_time      (){return 0;}
 virtual double get_time_accel(){return 0;}
 virtual void   set_time_accel(double ta){}

 virtual void write_scn_line(FILEHANDLE scn,char *item,char *string){}
 virtual bool	read_scn_line (FILEHANDLE scn,char *&line){return false;}

 virtual VESSEL   *vessel_by_name(char *name){return NULL;}
 virtual OBJHANDLE object_by_name(char *name){return 0;}
 virtual int       get_cb_count(){return 0;}
 virtual OBJHANDLE cb_by_index(int n){return 0;}
 virtual VESSEL   *get_focus_ves(){return NULL;}
 virtual char     *get_obj_name(OBJHANDLE r){return NULL;}
 virtual OBJHANDLE get_ves_obj(VESSEL *us){return 0;}

 virtual VECTOR3   get_angular_vel   (VESSEL *us){return tvec(0,0,0);}
 virtual OBJHANDLE get_gravity_ref   (VESSEL *us){return 0;}
 virtual OBJHANDLE get_surface_ref   (VESSEL *us){return 0;}
 virtual bool      ves_ground_contact(VESSEL *us){return false;}

 virtual double  get_obj_size      (OBJHANDLE r){return 0;}
 virtual double  get_obj_mass      (OBJHANDLE r){return 0;}
 virtual double  get_obj_altitude  (OBJHANDLE p,OBJHANDLE ref){return 0;}
 virtual VECTOR3 get_global_obj_pos(OBJHANDLE r){return tvec(0,0,0);}
 virtual VECTOR3 get_global_obj_vel(OBJHANDLE r){return tvec(0,0,0);}
 virtual MATRIX3 get_obj_rot_matrix(OBJHANDLE p){return tmat(0,0,0,0,0,0,0,0,0);}
 virtual double  get_planet_rotrate(OBJHANDLE p){return 0;}
 virtual MATRIX3 planet_horizon_matrix(OBJHANDLE p,VECTOR3 pos){return tmat(0,0,0,0,0,0,0,0,0);}

 virtual double  get_ves_mass   (VESSEL *us){return 0;}
 virtual double  get_ves_size   (VESSEL *us){return 0;}
 virtual VECTOR3 get_ves_pmi    (VESSEL *us){return tvec(0,0,0);}
 virtual char   *get_ves_name   (VESSEL *us){return NULL;}
 virtual void    get_ves_equ_pos(VESSEL *us,double &longitude,double &latitude,double &radius){}

 virtual VECTOR3   get_relative_vel  (VESSEL *us,OBJHANDLE r){return tvec(0,0,0);}
 virtual VECTOR3   get_relative_pos  (VESSEL *us,OBJHANDLE r){return tvec(0,0,0);}
 virtual VECTOR3   get_global_vel    (VESSEL *us){return tvec(0,0,0);}
 virtual VECTOR3   get_global_pos    (VESSEL *us){return tvec(0,0,0);}

 virtual void    set_airsurface_level           (VESSEL *us,AIRCTRL_TYPE type,double lv){}
 virtual void    af_control_set                 (VESSEL *us,int state){}

 virtual VECTOR3 get_ves_airspeed_vector        (VESSEL *us){return tvec(0,0,0);}
 virtual VECTOR3 get_ves_horizon_airspeed_vector(VESSEL *us){return tvec(0,0,0);}
 virtual double  get_ves_altitude(VESSEL *us){return 0;}
 virtual VECTOR3 get_obj_relative_vel(OBJHANDLE p,OBJHANDLE ref){return tvec(0,0,0);}
 virtual VECTOR3 get_obj_relative_pos(OBJHANDLE p,OBJHANDLE ref){return tvec(0,0,0);}


 virtual VECTOR3 horizon_rot   (VESSEL *us,const VECTOR3 rloc){return tvec(0,0,0);}
 virtual VECTOR3 rot_horizon   (VESSEL *us,const VECTOR3 rloc){return tvec(0,0,0);}
 virtual VECTOR3 global_2_local(VESSEL *us,const VECTOR3 rloc){return tvec(0,0,0);}
 virtual VECTOR3 local_2_global(VESSEL *us,const VECTOR3 rloc){return tvec(0,0,0);}

 virtual int             group_thruster_count    (VESSEL *us,THGROUP_TYPE g){return 0;}
 virtual THRUSTER_HANDLE group_thruster          (VESSEL *us,THGROUP_TYPE g,int i){return 0;}
 virtual void            set_thruster_group_level(VESSEL *us,THGROUP_TYPE g,double lv){}
 virtual double          get_thruster_group_level(VESSEL *us,THGROUP_TYPE g){return 0;}

 virtual int             thruster_count    (VESSEL *us){return 0;}
 virtual THRUSTER_HANDLE thruster_by_index (VESSEL *us,int i){return 0;}
 virtual double          get_thruster_level(VESSEL *us,THRUSTER_HANDLE th){return 0;}
 virtual void            set_thruster_level(VESSEL *us,THRUSTER_HANDLE th,double lv){}
 virtual OBJHANDLE       thruster_fuel     (VESSEL *us,THRUSTER_HANDLE th){return 0;}
 virtual double          thruster_isp      (VESSEL *us,THRUSTER_HANDLE th){return 0;}
 virtual double          fuel_efficiency   (VESSEL *us,OBJHANDLE pr){return 0;}
 virtual VECTOR3         thruster_dir      (VESSEL *us,THRUSTER_HANDLE th){return tvec(0,0,0);}
 virtual double          thruster_max0     (VESSEL *us,THRUSTER_HANDLE th){return 0;}
 virtual void            thruster_moment   (VESSEL *us,THRUSTER_HANDLE th,VECTOR3 &f,VECTOR3 &t){}
 virtual VECTOR3         thruster_ref      (VESSEL *us,THRUSTER_HANDLE th){return tvec(0,0,0);}  

 virtual DOCKHANDLE get_dock   (VESSEL *us,int port){return 0;}
 virtual UINT       dock_status(VESSEL *us,int port){return 0;}
 virtual void       dock_params(VESSEL *us,DOCKHANDLE hDock,VECTOR3 &pos,VECTOR3 &dir,VECTOR3 &rot){}   
                               
 virtual void ves_key_press(VESSEL *us,int key){}
 virtual double get_obj_equ_inclination(OBJHANDLE p,OBJHANDLE ref){return 0;}
 virtual VECTOR3 sph_2_rec(OBJHANDLE p,double vlat,double vlon,double vrad){return tvec(0,0,0);}
 virtual double calculate_azimuth(VESSEL *ves,double tgt_radius,double tgt_inc){return 0;}
 virtual THGROUP_TYPE int2thgroup_type(int tg){return THGROUP_USER;}
 virtual int thgroup_type2int(THGROUP_TYPE tg){return 0;} 
};                       
//#######################################################################################//
#endif
//#######################################################################################//