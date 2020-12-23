//#######################################################################################//
//UAP MFD, Made by Artlav in 2011
//The UI
//#######################################################################################//
#define _CRT_SECURE_NO_WARNINGS 1    
#define _CRT_SECURE_NO_DEPRECATE 1 
#define _CRT_NONSTDC_NO_DEPRECATE 1 
#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
//#######################################################################################//
//Removed various leftover debug output files

//More keys in tools!
//Ascending and descending - targetting
//Add wait for target path crossing in take-off
//Bug: Lunar man, del all, add 2 attitudes, only in release
//Add MFD scroll list
//Altitude awareness in lift-off, for launch profile approximations
//#######################################################################################//
#define ORBITER_MODULE
#include "sal.h"
#include "iap.h"
#include "uap.h"
#include "uap_planner.h"
#include "uapsys.h"
#include "util.h"
//#######################################################################################//
DLLCLBK void opcDLLInit(HINSTANCE hDLL)
{
 static char *uap_name="UAP MFD";
 MFDMODESPEC uap_spec;
 uap_spec.name=uap_name;
 uap_spec.key=OAPI_KEY_B;
 uap_spec.msgproc=uapmfd::MsgProc;
 uap_mfdmode=oapiRegisterMFDMode(uap_spec);
 
 static char *uap_planner_name="UAP Planner MFD";
 MFDMODESPEC uap_planner_spec;
 uap_planner_spec.name=uap_planner_name;
 uap_planner_spec.key=OAPI_KEY_N;
 uap_planner_spec.msgproc=uap_planner_mfd::MsgProc;
 //uap_planner_mfdmode=oapiRegisterMFDMode(uap_planner_spec);
 
 uap_init();
}
//#######################################################################################//
DLLCLBK void opcDLLExit(HINSTANCE hDLL){oapiUnregisterMFDMode(uap_mfdmode);/*oapiUnregisterMFDMode(uap_planner_mfdmode);*/}
//#######################################################################################//
DLLCLBK void opcPreStep(double SimT,double SimDT,double mjd){uap_step(SimT,SimDT,mjd);}
DLLCLBK void opcSaveState(FILEHANDLE scn){uap_save(scn);}
DLLCLBK void opcLoadState(FILEHANDLE scn){uap_load(scn);}
//#######################################################################################//
//#######################################################################################//
uapmfd::uapmfd(DWORD w,DWORD h,VESSEL *vessel):MFD(w,h,vessel)
{
 mfd_seq=-1;
 scrx=w;
 scry=h;
 page=0;
 sel=0;
 sels=0;
 can_enter=0;
 ap_n=0;
}
//#######################################################################################//
uapmfd::~uapmfd(){}
//#######################################################################################//
char *uapmfd::ButtonLabel(int bt)
{
 static char *label[12]={"STP","PG"," < "," > ","ENT","GO","PRV","NXT","INS","DEL"," ^ "," v "};
 return(bt<12?label[bt]:0);
}
//#######################################################################################//
int uapmfd::ButtonMenu(const MFDBUTTONMENU **menu) const
{
 static const MFDBUTTONMENU mnu[12]={  
  {"   Some are not resumable",
   "<- Stop",'S'}, 
  {"   ",
   "<- Switch page",'P'}, 
  {"   ",
   "<- Prev var",'<'},
  {"   ",
   "<- Next var",'>'},
  {"   ",
   "<- Enter variable",'E'},
  {"   ",
   "<- Start sequence",'G'},
  {"",
   "Prev AP ->",'R'},
  {"",
   "Next AP ->",'F'},
  {"",
   "Insert AP ->",'I'},
  {"",
   "Delete AP ->",'D'},
  {"",
   "Move AP up ->",'['},
  {"",
   "Move AP dwn ->",']'},
 };
 if(menu)*menu=mnu;
 return 12;
}
//#######################################################################################//
//#######################################################################################//
bool uapmfd::ConsumeButton(int bt,int event)
{
 if(!(event & PANEL_MOUSE_LBDOWN))return false;
 switch(bt){                                                
  case  0:ConsumeKeyBuffered(OAPI_KEY_S);break;
  case  1:ConsumeKeyBuffered(OAPI_KEY_P);break;          
  case  2:ConsumeKeyBuffered(OAPI_KEY_COMMA);break;
  case  3:ConsumeKeyBuffered(OAPI_KEY_PERIOD);break;   
  case  4:ConsumeKeyBuffered(OAPI_KEY_E);break;
  case  5:ConsumeKeyBuffered(OAPI_KEY_G);break;   
  case  6:ConsumeKeyBuffered(OAPI_KEY_R);break;
  case  7:ConsumeKeyBuffered(OAPI_KEY_F);break;
  case  8:ConsumeKeyBuffered(OAPI_KEY_I);break;
  case  9:ConsumeKeyBuffered(OAPI_KEY_D);break;
  case 10:ConsumeKeyBuffered(OAPI_KEY_LBRACKET);break;
  case 11:ConsumeKeyBuffered(OAPI_KEY_RBRACKET);break;
  default:return false;
 }
 return true;
}
//#######################################################################################//
bool uapmfd::ConsumeKeyImmediate(char *kstate){return false;}
bool uapmfd::ConsumeKeyBuffered(DWORD key)
{          
 switch(key){   
  case OAPI_KEY_T:if(uap_set_time_accel<100000)uap_set_time_accel=uap_set_time_accel*10;return true;
  case OAPI_KEY_A:uap_time_acc=(uap_time_acc+1)%3;uap_set_time_accel=1;return true;
  case OAPI_KEY_P:uap_clear_error();page=(page+1)%pages;return true;
  case OAPI_KEY_PERIOD:if(sels)sel=(sel+1)%sels;else sel=0;return true;
  case OAPI_KEY_COMMA:sel=sel-1;if(sel<0)sel=sels-1;return true;
  case OAPI_KEY_E:do_enter();return true;  
  case OAPI_KEY_I:seq_op(0,0);return true;
  case OAPI_KEY_D:seq_op(1,0);return true;
  case OAPI_KEY_LBRACKET:seq_op(2,0);return true;
  case OAPI_KEY_RBRACKET:seq_op(2,1);return true;
  case OAPI_KEY_G:{get_tgt();uap_start_seq(mfd_seq);if(mfd_seq!=-1)page=0;return true; }
  case OAPI_KEY_S:{get_tgt();uap_stop_seq(mfd_seq);return true; }
  case OAPI_KEY_F:{get_tgt();if(mfd_seq!=-1)if(seqs[mfd_seq].cnt)ap_n=(ap_n+1)%seqs[mfd_seq].cnt;else ap_n=0;return true;}
  case OAPI_KEY_R:{get_tgt();if(mfd_seq!=-1){ap_n=(ap_n-1);if(ap_n<0)ap_n=seqs[mfd_seq].cnt-1;}return true;}
  case OAPI_KEY_Q:ExitProcess(0);return true; 
  case OAPI_KEY_V:get_tgt();test_seq_make();return true; 
  default:return false;
 }
}
//#######################################################################################//
void uapmfd::seq_op(int op,int tp)
{
 VESSEL *ves=oapiGetFocusInterface();
 iautopilot *ap;
 uap_clear_error();
 
 if(page<2)return;
 get_tgt();
 
 switch(op){
  case 0:page=3;break;
  case 3:page=2;if((sel<0)||(sel>=lib_cnt))return;if(!(ap=get_lib_obj(libs[sel].id,ves)))return;uap_add_seq_ap(mfd_seq,ap_n,ves,ap);break;
  case 1:uap_del_seq_step(mfd_seq,ap_n);sel=0;break;
  case 2:uap_move_seq_step(mfd_seq,ap_n,tp);break;
 }
}
//#######################################################################################//
bool cb_set(void *id,char *str,void *data){return(((uapmfd *)data)->set_cur(str));}
//#######################################################################################//
void uapmfd::do_enter()
{
 if(!can_enter)return;
 char buf[255];
 int c,i;
 iautopilot *ap;
 
 if(page==1){
  get_tgt();
  if(mfd_seq==-1)return;
  if((ap_n>=seqs[mfd_seq].cnt)||(ap_n==-1))return;
  ap=seqs[mfd_seq].seq[ap_n].ap;
  c=0;for(i=0;i<ap->var_cnt;i++)if(!(ap->vars[i].flags&VF_STATE))if(!(ap->vars[i].flags&VF_OUT)){if(sel==c){c=i;break;}c++;}
  if(c>=ap->var_cnt)return;
  
  if(ap->vars[i].typ==VT_INT){
   if(!strcmp(ap->vars[i].name,"engine"))sprintf(buf,"Input %s (%d..%d,main,hover,retro,maininv)",ap->vars[i].name,(int)ap->vars[i].min_val,(int)ap->vars[i].max_val);
                                    else sprintf(buf,"Input %s (%d..%d)",ap->vars[i].name,(int)ap->vars[i].min_val,(int)ap->vars[i].max_val);
  }
  if(ap->vars[i].typ==VT_DBL){
   if(ap->vars[i].flags&VF_DEG)sprintf(buf,"Input %s (%f..%f)",ap->vars[i].name,ap->vars[i].min_val/RAD,ap->vars[i].max_val/RAD);
                          else sprintf(buf,"Input %s (%f..%f)",ap->vars[i].name,ap->vars[i].min_val,ap->vars[i].max_val);
  }
  if(ap->vars[i].typ==VT_CHAR)sprintf(buf,"Input %s",ap->vars[i].name);
  oapiOpenInputBox(buf,cb_set,0,20,(void*)this);
 }else if(page==3)seq_op(3,0);
}
//#######################################################################################//
bool uapmfd::set_cur(char *in)
{
 int c,i;
 iautopilot *ap;
 
 get_tgt();
 if(mfd_seq==-1)return false;
 if((ap_n>=seqs[mfd_seq].cnt)||(ap_n==-1))return false;
 ap=seqs[mfd_seq].seq[ap_n].ap;
 c=0;for(i=0;i<ap->var_cnt;i++)if(!(ap->vars[i].flags&VF_STATE))if(!(ap->vars[i].flags&VF_OUT)){if(sel==c){c=i;break;}c++;}
 if(c>=ap->var_cnt)return false;
 
 switch(ap->vars[c].typ){
  case VT_DBL:{
   double par=strtod(in,NULL);
   if(ap->vars[c].flags&VF_DEG)par=par*RAD;
   ap->set_dbl_var(ap->vars[c].name,par);
   break;
  }
  case VT_INT:ap->set_int_var(ap->vars[c].name,const_test(in));break;
  case VT_CHAR:ap->set_char_var(ap->vars[c].name,in);break;
  default:return false;
 } 
 
 return true;
}
//#######################################################################################//
void uapmfd::get_tgt()
{
 VESSEL *ves=oapiGetFocusInterface();
 if(mfd_seq>=seq_cnt)mfd_seq=-1;
 if(mfd_seq!=-1)if(!seqs[mfd_seq].used)mfd_seq=-1;
 if(mfd_seq!=-1)if(seqs[mfd_seq].us!=ves)mfd_seq=-1;
 if(mfd_seq==-1)mfd_seq=get_seq_by_vessel(ves);
}
//#######################################################################################//
void uapmfd::write_vars(HDC hDC,int &line,iautopilot *ap,int use_sel)
{
 int i;
 char s[100],v[100];
 
 can_enter=use_sel;
 ap->info();
 
 sels=0;
 for(i=0;i<ap->var_cnt;i++)if(!(ap->vars[i].flags&VF_STATE)){
  if(ap->vars[i].flags&VF_OUT)SetTextColor(hDC,RGB(255,255,128));
                         else SetTextColor(hDC,RGB(128,255,128));
  switch(ap->vars[i].typ){
   case VT_DBL:sprintf(v,"%f",ap->vars[i].flags&VF_DEG?*((double*)ap->vars[i].var)/RAD:*((double*)ap->vars[i].var));break;
   case VT_INT:{
    if(!strcmp(ap->vars[i].name,"engine"))switch(*((int*)ap->vars[i].var)){
     case 1:sprintf(v,"main");break;
     case 2:sprintf(v,"retro");break;
     case 4:sprintf(v,"hover");break;
     case 8:sprintf(v,"maininv");break;
     default:sprintf(v,"%d",*((int*)ap->vars[i].var));
    }else if((!strcmp(ap->vars[i].name,"key"))||(!strcmp(ap->vars[i].name,"gear_key")))switch(*((int*)ap->vars[i].var)){
     case OAPI_KEY_G:sprintf(v,"g");break;
     case OAPI_KEY_J:sprintf(v,"j");break;
     case OAPI_KEY_K:sprintf(v,"k");break;
     case OAPI_KEY_F:sprintf(v,"f");break;
     case OAPI_KEY_E:sprintf(v,"e");break;
     case OAPI_KEY_O:sprintf(v,"o");break;
     default:sprintf(v,"%d",*((int*)ap->vars[i].var));
    }else sprintf(v,"%d",*((int*)ap->vars[i].var));
    break;
   }
   case VT_CHAR:sprintf(v,"%s",(char*)ap->vars[i].var);break;
   default:sprintf(v,"Unknown");
  }
  if(use_sel&&(sel==sels)){SetTextColor(hDC,RGB(200,200,200));sprintf(s,">%s = %s",ap->vars[i].name,v);print(hDC,10,&line,s);}
                     else {                                   sprintf(s," %s = %s",ap->vars[i].name,v);print(hDC,10,&line,s);}
  if(!(ap->vars[i].flags&VF_OUT))sels++;
 }
 if(sel>=sels)sel=sels-1;if(sel<0)sel=0;
}
//#######################################################################################//
void uapmfd::write_seqs(HDC hDC,int &line,int n,int use_sel)
{
 int i;
 char s[100];
 
 sels=0;
 for(i=0;i<seqs[n].cnt;i++){
  SetTextColor(hDC,RGB(128,255,128));
  if(use_sel&&(ap_n==i)){SetTextColor(hDC,RGB(200,200,200));sprintf(s,">%s",seqs[n].seq[i].ap->name);print(hDC,10,&line,s);}
                   else {                                   sprintf(s," %s",seqs[n].seq[i].ap->name);print(hDC,10,&line,s);}
  sels++;
 }
 if(ap_n>=sels)ap_n=sels-1;if(ap_n<0)ap_n=0;
 
}
//#######################################################################################//
void uapmfd::write_avls(HDC hDC,int &line,int use_sel)
{
 int i;
 char s[100];
 
 can_enter=use_sel;
 sels=0;
 for(i=0;i<lib_cnt;i++){
  SetTextColor(hDC,RGB(128,255,128));
  if(use_sel&&(sel==i)){SetTextColor(hDC,RGB(200,200,200));sprintf(s,">%s",libs[i].id);print(hDC,10,&line,s);}
                   else{                                   sprintf(s," %s",libs[i].id);print(hDC,10,&line,s);}
  sels++;
 }
 if(ap_n>=sels)ap_n=sels-1;if(ap_n<0)ap_n=0;
}
//#######################################################################################//
void uapmfd::write_error(HDC hDC,int &line)
{
 char s[100];
 SetTextColor(hDC,RGB(255,128,128));
 line+=10;
 sprintf(s,"%s",uap_error);print(hDC,10,&line,s);
 sprintf(s,"(PG to exit)",uap_error);print(hDC,10,&line,s);
}
//#######################################################################################//
void uapmfd::Update(HDC hDC)
{
 can_enter=0;
 VESSEL *ves=oapiGetFocusInterface();
 get_tgt();

 if(uap_fail){
  uap_fail=0;
  page=4;
 }
 
 int line=0;
 char s[100];
 Title(hDC,"Universal autopilots control (v0.3.1)");
 switch(page){
  case 0:sprintf(s,"Page %d/%d  Status",page,pages-1);break;
  case 1:sprintf(s,"Page %d/%d  Input",page,pages-1);break;
  case 2:sprintf(s,"Page %d/%d  Sequence",page,pages-1);break;
  case 3:sprintf(s,"Select AP to insert");break;
  case 4:SetTextColor(hDC,RGB(255,128,128));sprintf(s,"Sequence error");break;
 }print(hDC,10,&line,s);
 
 SetTextColor(hDC,RGB(128,255,128));
 sprintf(s,"In %s (%s)",ves->GetName(),ves->GetClassName());print(hDC,10,&line,s);
  
 if(page==0){
  if(mfd_seq==-1){
   sprintf(s,"No sequence defined for it");print(hDC,10,&line,s);
   SetTextColor(hDC,RGB(255,255,128));
   sprintf(s,"To define, go to page 2.");print(hDC,10,&line,s);
  }else{
   if(seqs[mfd_seq].run){
    SetTextColor(hDC,RGB(255,0,0));
    sprintf(s,"Running sequence");print(hDC,10,&line,s);
    if(!uap_time_acc){SetTextColor(hDC,RGB((255*(sin(oapiGetSimTime()*4)+1)/2),(255*(cos(oapiGetSimTime()*4)+1)/2),0));sprintf(s,"Time accel control OFF");print(hDC,10,&line,s);SetTextColor(hDC,RGB(255,0,0));}
    if(uap_time_acc==2){SetTextColor(hDC,RGB((255*(sin(oapiGetSimTime()*4)+1)/2),0,(255*(cos(oapiGetSimTime()*4)+1)/2)));sprintf(s,"Time accel=%f",uap_set_time_accel);print(hDC,10,&line,s);SetTextColor(hDC,RGB(255,0,0));}
    sprintf(s,"Step %d of %d, %s",seqs[mfd_seq].cur,seqs[mfd_seq].cnt-1,seqs[mfd_seq].seq[seqs[mfd_seq].cur].ap->name);print(hDC,10,&line,s);
    SetTextColor(hDC,RGB(255,128,0));
    sprintf(s,"Now: %s",seqs[mfd_seq].seq[seqs[mfd_seq].cur].ap->state_string);print(hDC,10,&line,s);
   }else{sprintf(s,"Not running sequence");print(hDC,10,&line,s);}  
   SetTextColor(hDC,RGB(128,255,128));
   if((seqs[mfd_seq].cur<seqs[mfd_seq].cnt)&&(seqs[mfd_seq].cur!=-1)){
    if(seqs[mfd_seq].run){
     write_vars(hDC,line,seqs[mfd_seq].seq[seqs[mfd_seq].cur].ap,0);
    }else{
     SetTextColor(hDC,RGB(255,60,60));
     sprintf(s,"Press GO to start sequence");print(hDC,10,&line,s); 
    }
   }else{
    sprintf(s,"Non-started/Finished/Uninitialized");print(hDC,10,&line,s); 
   }
  }
 }
 if(page==1){
  if(mfd_seq==-1){
   sprintf(s,"No sequence defined for it");print(hDC,10,&line,s);
   SetTextColor(hDC,RGB(255,255,128));
   sprintf(s,"To define, go to page 2.");print(hDC,10,&line,s);
  }else if((ap_n<seqs[mfd_seq].cnt)&&(ap_n>=0)){
   sprintf(s,"Step %d of %d, %s",ap_n,seqs[mfd_seq].cnt-1,seqs[mfd_seq].seq[ap_n].ap->name);print(hDC,10,&line,s);
   if((seqs[mfd_seq].run)&&(ap_n==seqs[mfd_seq].cur)){
    SetTextColor(hDC,RGB(255,0,0)); 
    sprintf(s,"Step is running");print(hDC,10,&line,s);
    SetTextColor(hDC,RGB(128,255,128));
    sprintf(s,"So input is read-only");print(hDC,10,&line,s);
    sprintf(s,"(On page 0)");print(hDC,10,&line,s);
   }else if((seqs[mfd_seq].run)&&(ap_n<seqs[mfd_seq].cur)){
    SetTextColor(hDC,RGB(255,255,0)); 
    sprintf(s,"Step is done");print(hDC,10,&line,s);
    SetTextColor(hDC,RGB(128,255,128));
    sprintf(s,"So input is irrelevant");print(hDC,10,&line,s);
   }else write_vars(hDC,line,seqs[mfd_seq].seq[ap_n].ap,1);
  }
 }
 if(page==2){
  if(mfd_seq==-1){
   SetTextColor(hDC,RGB(255,255,128));
   sprintf(s,"Sequence is empty");print(hDC,10,&line,s);
   sprintf(s,"Use INS to add steps");print(hDC,10,&line,s);
  }else write_seqs(hDC,line,mfd_seq,1);
 }
 if(page==3)write_avls(hDC,line,1);
 if(page==4)write_error(hDC,line);
}
//#######################################################################################//
int uapmfd::MsgProc(UINT msg,UINT mfd,WPARAM wparam,LPARAM lparam){switch(msg){case OAPI_MSG_MFD_OPENED:return(int)(new uapmfd(LOWORD(wparam),HIWORD(wparam),(VESSEL*)lparam));}return 0;}
//#######################################################################################//
