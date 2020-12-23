//#######################################################################################//
//UAP module, Made by Artlav in 2011
//Orbital maneuvring autopilots
//#######################################################################################//
class ap_align:public iautopilot{
public:
 ap_align(SAL *sal_in,VESSEL *vessel);
 void start();
 void stop();
 void step(double simt,double simdt);
 void init();

 OBJHANDLE tgt_obj;
 char tgt_nam[255];
 double rinc_delta;
 int use_grp;
 
 double rinc,time_to_plane,burn_time;
};
//#######################################################################################//
class ap_approach:public iautopilot{
public:
 ap_approach(SAL *sal_in,VESSEL *vessel);
 void start();
 void stop();
 void step(double simt,double simdt);
 void init();

 VESSEL *tgt_ves;
 char tgt_nam[255];
 double max_velocity,tgt_distance,cur_distance;
 int use_grp,corr;
};
//#######################################################################################//
class ap_sync_orbit:public iautopilot{
public:
 ap_sync_orbit(SAL *sal_in,VESSEL *vessel);
 void start();
 void stop();
 void step(double simt,double simdt);
 void init();
 void info();
 void reset();

 VESSEL *tgt_ves;
 char tgt_nam[255];
 double cur_distance,tgt_distance,max_dv_to_use,dt,tt,rt,dv;
 int use_grp,stage,minimize;
 char minimizing_by[255];
 
 double mindp,adv[100],adt[100],adp[100];
 int was,ws_cnt,s,ws_cur;
};
//#######################################################################################//
class ap_hohmann:public iautopilot{
public:
 ap_hohmann(SAL *sal_in,VESSEL *vessel);
 void start();
 void stop();
 void step(double simt,double simdt);
 void init();

 OBJHANDLE tgt_obj;
 char tgt_nam[255];
 double tgt_orbit_alt;
 int use_grp,stage,dir;
 
 double dt,dv,tgt_apd,g_tgt;
 VECTOR3 ttp;
};
//#######################################################################################//
iautopilot *mk_align(SAL *sal_in,VESSEL *v);
iautopilot *mk_approach(SAL *sal_in,VESSEL *v);
iautopilot *mk_sync_orbit(SAL *sal_in,VESSEL *v);
iautopilot *mk_hohmann(SAL *sal_in,VESSEL *v);
//#######################################################################################//
