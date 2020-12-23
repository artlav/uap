//#######################################################################################//
//UAP core, Made by Artlav in 2011
//Simulator Abstraction Layer, class definitions
//#######################################################################################//
#include "sal.h"  
//#######################################################################################//
class SAL_IMP:public SAL{
public:
 double sim_time      ();
 double get_time_accel();
 void   set_time_accel(double ta);

 void write_scn_line(FILEHANDLE scn,char *item,char *string);
 bool	read_scn_line (FILEHANDLE scn,char *&line);

 VESSEL   *vessel_by_name(char *name);
 OBJHANDLE object_by_name(char *name);
 int       get_cb_count();
 OBJHANDLE cb_by_index(int n);
 VESSEL   *get_focus_ves();
 char     *get_obj_name(OBJHANDLE r);
 OBJHANDLE get_ves_obj(VESSEL *us);

 VECTOR3   get_angular_vel   (VESSEL *us);
 OBJHANDLE get_gravity_ref   (VESSEL *us);
 OBJHANDLE get_surface_ref   (VESSEL *us);
 bool      ves_ground_contact(VESSEL *us);

 double  get_obj_size      (OBJHANDLE r);
 double  get_obj_mass      (OBJHANDLE r);
 double  get_obj_altitude  (OBJHANDLE p,OBJHANDLE ref);
 VECTOR3 get_global_obj_pos(OBJHANDLE r);
 VECTOR3 get_global_obj_vel(OBJHANDLE r);
 MATRIX3 get_obj_rot_matrix(OBJHANDLE p);
 double  get_planet_rotrate(OBJHANDLE p);
 MATRIX3 planet_horizon_matrix(OBJHANDLE p,VECTOR3 pos);

 double  get_ves_mass   (VESSEL *us){return get_obj_mass(get_ves_obj(us));}
 double  get_ves_size   (VESSEL *us){return get_obj_size(get_ves_obj(us));}
 VECTOR3 get_ves_pmi    (VESSEL *us);
 char   *get_ves_name   (VESSEL *us);
 void    get_ves_equ_pos(VESSEL *us,double &longitude,double &latitude,double &radius);

 VECTOR3   get_relative_vel  (VESSEL *us,OBJHANDLE r){return get_global_obj_vel(get_ves_obj(us))-get_global_obj_vel(r);}
 VECTOR3   get_relative_pos  (VESSEL *us,OBJHANDLE r){return get_global_obj_pos(get_ves_obj(us))-get_global_obj_pos(r);}
 VECTOR3   get_global_vel    (VESSEL *us){return get_global_obj_vel(get_ves_obj(us));}
 VECTOR3   get_global_pos    (VESSEL *us){return get_global_obj_pos(get_ves_obj(us));}

 void    set_airsurface_level           (VESSEL *us,AIRCTRL_TYPE type,double lv);
 void    af_control_set                 (VESSEL *us,int state);

 VECTOR3 get_ves_airspeed_vector        (VESSEL *us);
 VECTOR3 get_ves_horizon_airspeed_vector(VESSEL *us);
 double  get_ves_altitude(VESSEL *us){return get_obj_altitude(get_ves_obj(us),get_surface_ref(us));}
 VECTOR3 get_obj_relative_vel(OBJHANDLE p,OBJHANDLE ref){return get_global_obj_vel(p)-get_global_obj_vel(ref);}
 VECTOR3 get_obj_relative_pos(OBJHANDLE p,OBJHANDLE ref){return get_global_obj_pos(p)-get_global_obj_pos(ref);}


 VECTOR3 horizon_rot   (VESSEL *us,const VECTOR3 rloc);
 VECTOR3 rot_horizon   (VESSEL *us,const VECTOR3 rloc);
 VECTOR3 global_2_local(VESSEL *us,const VECTOR3 rloc);
 VECTOR3 local_2_global(VESSEL *us,const VECTOR3 rloc);

 int             group_thruster_count    (VESSEL *us,THGROUP_TYPE g);
 THRUSTER_HANDLE group_thruster          (VESSEL *us,THGROUP_TYPE g,int i);
 void            set_thruster_group_level(VESSEL *us,THGROUP_TYPE g,double lv);
 double          get_thruster_group_level(VESSEL *us,THGROUP_TYPE g);

 int             thruster_count    (VESSEL *us);
 THRUSTER_HANDLE thruster_by_index (VESSEL *us,int i);
 double          get_thruster_level(VESSEL *us,THRUSTER_HANDLE th);
 void            set_thruster_level(VESSEL *us,THRUSTER_HANDLE th,double lv);
 OBJHANDLE       thruster_fuel     (VESSEL *us,THRUSTER_HANDLE th);
 double          thruster_isp      (VESSEL *us,THRUSTER_HANDLE th);
 double          fuel_efficiency   (VESSEL *us,OBJHANDLE pr);
 VECTOR3         thruster_dir      (VESSEL *us,THRUSTER_HANDLE th);
 double          thruster_max0     (VESSEL *us,THRUSTER_HANDLE th);
 void            thruster_moment   (VESSEL *us,THRUSTER_HANDLE th,VECTOR3 &f,VECTOR3 &t);
 VECTOR3         thruster_ref      (VESSEL *us,THRUSTER_HANDLE th);  

 DOCKHANDLE get_dock   (VESSEL *us,int port);
 UINT       dock_status(VESSEL *us,int port);
 void       dock_params(VESSEL *us,DOCKHANDLE hDock,VECTOR3 &pos,VECTOR3 &dir,VECTOR3 &rot);   
                               
 void ves_key_press(VESSEL *us,int key);
 double get_obj_equ_inclination(OBJHANDLE p,OBJHANDLE ref);
 VECTOR3 sph_2_rec(OBJHANDLE p,double vlat,double vlon,double vrad);
 double calculate_azimuth(VESSEL *ves,double tgt_radius,double tgt_inc);
 THGROUP_TYPE int2thgroup_type(int tg);
 int thgroup_type2int(THGROUP_TYPE tg); 
};                       
//#######################################################################################//
#ifdef WIN32
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT
#endif
extern "C" {
DLLEXPORT void  uapd_set(void *gm,void *gv,void *gd,void *gi,void *sv,void *sd,void *si);
}
//#######################################################################################//
