//#######################################################################################//
//UAP core, Made by Artlav in 2011
//Simulator Abstraction Layer, Orbiter implementation
//#######################################################################################//
double SAL_IMP::sim_time      ()         {return oapiGetSimTime();}
double SAL_IMP::get_time_accel()         {return oapiGetTimeAcceleration();}
void   SAL_IMP::set_time_accel(double ta){       oapiSetTimeAcceleration(ta);}
//#######################################################################################//
void SAL_IMP::write_scn_line(FILEHANDLE scn,char *item,char *string){oapiWriteScenario_string(scn,item,string);}
bool SAL_IMP::read_scn_line (FILEHANDLE scn,char *&line){return oapiReadScenario_nextline(scn,line);}
//#######################################################################################//
VESSEL   *SAL_IMP::vessel_by_name(char *name) {OBJHANDLE t=oapiGetVesselByName(name);if(!t)return NULL;return oapiGetVesselInterface(t);}
OBJHANDLE SAL_IMP::object_by_name(char *name) {return oapiGetObjectByName(name);}
int       SAL_IMP::get_cb_count  ()           {return oapiGetGbodyCount();}
OBJHANDLE SAL_IMP::cb_by_index   (int n)      {return oapiGetGbodyByIndex(n);}
VESSEL   *SAL_IMP::get_focus_ves ()           {return oapiGetFocusInterface();}
char     *SAL_IMP::get_obj_name  (OBJHANDLE r){static char n[255];oapiGetObjectName(r,n,255);return n;}
OBJHANDLE SAL_IMP::get_ves_obj   (VESSEL *us) {return us->GetHandle();}
//#######################################################################################//
double  SAL_IMP::get_obj_size      (OBJHANDLE r){return oapiGetSize(r);}
double  SAL_IMP::get_obj_mass      (OBJHANDLE r){return oapiGetMass(r);}
VECTOR3 SAL_IMP::get_global_obj_pos(OBJHANDLE r){VECTOR3 v;oapiGetGlobalPos(r,&v);return v;}
VECTOR3 SAL_IMP::get_global_obj_vel(OBJHANDLE r){VECTOR3 v;oapiGetGlobalVel(r,&v);return v;}
MATRIX3 SAL_IMP::get_obj_rot_matrix(OBJHANDLE p){MATRIX3 gmat;oapiGetRotationMatrix(p,&gmat);return transpm(gmat);}
double  SAL_IMP::get_planet_rotrate(OBJHANDLE p){return oapiGetPlanetPeriod(p);}
//#######################################################################################//
VECTOR3 SAL_IMP::get_ves_pmi (VESSEL *us){VECTOR3 vec;us->GetPMI(vec);return vec;}
char   *SAL_IMP::get_ves_name(VESSEL *us){return us->GetName();}
//#######################################################################################//
VECTOR3   SAL_IMP::get_angular_vel   (VESSEL *us){VECTOR3 v;us->GetAngularVel(v);return v;}
OBJHANDLE SAL_IMP::get_gravity_ref   (VESSEL *us){return us->GetGravityRef();}
OBJHANDLE SAL_IMP::get_surface_ref   (VESSEL *us){return us->GetSurfaceRef();}
bool      SAL_IMP::ves_ground_contact(VESSEL *us){return us->GroundContact();}
//#######################################################################################//
void SAL_IMP::set_airsurface_level(VESSEL *us,AIRCTRL_TYPE type,double lv){us->SetControlSurfaceLevel(type,lv);}
void SAL_IMP::af_control_set      (VESSEL *us,int state){if(state)us->SetADCtrlMode(7);else us->SetADCtrlMode(0);}
//#######################################################################################//
int             SAL_IMP::group_thruster_count    (VESSEL *us,THGROUP_TYPE g)          {return us->GetGroupThrusterCount(g);}
THRUSTER_HANDLE SAL_IMP::group_thruster          (VESSEL *us,THGROUP_TYPE g,int i)    {return us->GetGroupThruster(g,i);}
void            SAL_IMP::set_thruster_group_level(VESSEL *us,THGROUP_TYPE g,double lv){       us->SetThrusterGroupLevel(g,lv);}
double          SAL_IMP::get_thruster_group_level(VESSEL *us,THGROUP_TYPE g)          {return us->GetThrusterGroupLevel(g);}
//#######################################################################################//
int             SAL_IMP::thruster_count    (VESSEL *us)                   {return us->GetThrusterCount();}
THRUSTER_HANDLE SAL_IMP::thruster_by_index (VESSEL *us,int i)             {return us->GetThrusterHandleByIndex(i);}
double          SAL_IMP::get_thruster_level(VESSEL *us,THRUSTER_HANDLE th){return us->GetThrusterLevel(th);}
void            SAL_IMP::set_thruster_level(VESSEL *us,THRUSTER_HANDLE th,double lv){us->SetThrusterLevel(th,lv);}
double          SAL_IMP::fuel_efficiency   (VESSEL *us,OBJHANDLE pr)      {return us->GetPropellantEfficiency(pr);}
OBJHANDLE       SAL_IMP::thruster_fuel     (VESSEL *us,THRUSTER_HANDLE th){return us->GetThrusterResource(th);}
double          SAL_IMP::thruster_isp      (VESSEL *us,THRUSTER_HANDLE th){return us->GetThrusterIsp(th);}
VECTOR3         SAL_IMP::thruster_dir      (VESSEL *us,THRUSTER_HANDLE th){VECTOR3 v;us->GetThrusterDir(th,v);return v;}
VECTOR3         SAL_IMP::thruster_ref      (VESSEL *us,THRUSTER_HANDLE th){VECTOR3 v;us->GetThrusterRef(th,v);return v;}
double          SAL_IMP::thruster_max0     (VESSEL *us,THRUSTER_HANDLE th){return us->GetThrusterMax0(th);}
void            SAL_IMP::thruster_moment   (VESSEL *us,THRUSTER_HANDLE th,VECTOR3 &f,VECTOR3 &t){us->GetThrusterMoment(th,f,t);}
//#######################################################################################//
DOCKHANDLE SAL_IMP::get_dock   (VESSEL *us,int port){return us->GetDockHandle(port);}
UINT       SAL_IMP::dock_status(VESSEL *us,int port){return us->DockingStatus(port);}
void       SAL_IMP::dock_params(VESSEL *us,DOCKHANDLE hDock,VECTOR3 &pos,VECTOR3 &dir,VECTOR3 &rot){us->GetDockParams(hDock,pos,dir,rot);}
//#######################################################################################//
void SAL_IMP::ves_key_press(VESSEL *us,int key)
{
 char keystatebuffer[0xFF],*keystate;
 for(int i=0;i<0xFF;i++)keystatebuffer[i]=0;
 keystate=&keystatebuffer[0];
 ((VESSEL2*)us)->clbkConsumeBufferedKey(key,true,keystate);
}
//#######################################################################################//
double SAL_IMP::get_obj_altitude(OBJHANDLE p,OBJHANDLE ref)
{
 return modv(get_global_obj_pos(ref)-get_global_obj_pos(p))-get_obj_size(ref);
}
//#######################################################################################//