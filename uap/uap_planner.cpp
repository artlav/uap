//#######################################################################################//
//UAP planner MFD, Made by Artlav in 2011
//I don't remember WTF that was supposed to be
//#######################################################################################//
#define _CRT_SECURE_NO_WARNINGS
//#######################################################################################//
#include "sal.h"
#include "iap.h"
#include "uap_planner.h"
#include "uapsys.h"
#include "../alibc/util.h"
//#######################################################################################//
uap_planner_mfd::uap_planner_mfd(DWORD w,DWORD h,VESSEL *vessel):MFD(w,h,vessel)
{
 mfd_seq=-1;
 scrx=w;
 scry=h;
}
//#######################################################################################//
uap_planner_mfd::~uap_planner_mfd(){}
//#######################################################################################//
char *uap_planner_mfd::ButtonLabel(int bt)
{
 static char *label[6]={">",">",">",">",">",">"};
 return(bt<6?label[bt]:0);
}
//#######################################################################################//
int uap_planner_mfd::ButtonMenu(const MFDBUTTONMENU **menu) const
{
 static const MFDBUTTONMENU mnu[6]={  
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
 };
 if(menu)*menu=mnu;
 return 6;
}
//#######################################################################################//
//#######################################################################################//
bool uap_planner_mfd::ConsumeButton(int bt,int event)
{
 if(!(event & PANEL_MOUSE_LBDOWN))return false;
 switch(bt){                                                
  case  0:ConsumeKeyBuffered(OAPI_KEY_1);break;
  case  1:ConsumeKeyBuffered(OAPI_KEY_2);break;          
  case  2:ConsumeKeyBuffered(OAPI_KEY_3);break;
  case  3:ConsumeKeyBuffered(OAPI_KEY_4);break;   
  case  4:ConsumeKeyBuffered(OAPI_KEY_5);break;
  case  5:ConsumeKeyBuffered(OAPI_KEY_6);break;   
  default:return false;
 }
 return true;
}
//#######################################################################################//
bool uap_planner_mfd::ConsumeKeyBuffered(DWORD key)
{
 switch(key){
  case OAPI_KEY_1:do_enter(1);return true;
  case OAPI_KEY_2:do_enter(2);return true;
  case OAPI_KEY_3:do_enter(3);return true;
  case OAPI_KEY_4:do_enter(4);return true;
  case OAPI_KEY_5:do_enter(5);return true;
  case OAPI_KEY_6:do_enter(6);return true;
  default:return false;
 }
}
//#######################################################################################//
void uap_planner_mfd::get_tgt()
{
 VESSEL *ves=oapiGetFocusInterface();
 if(mfd_seq>=seq_cnt)mfd_seq=-1;
 if(mfd_seq!=-1)if(!seqs[mfd_seq].used)mfd_seq=-1;
 if(mfd_seq!=-1)if(seqs[mfd_seq].us!=ves)mfd_seq=-1;
 if(mfd_seq==-1)mfd_seq=get_seq_by_vessel(ves);
}
//#######################################################################################//
bool cb_tgt_set(void *id,char *str,void *data){return(((uap_planner_mfd *)data)->do_set(str));}
//#######################################################################################//
void uap_planner_mfd::do_enter(int n)
{

 
  //oapiOpenInputBox(buf,cb_tgt_set,0,20,(void*)this);

}
//#######################################################################################//
bool uap_planner_mfd::do_set(char *nam)
{

 
  //oapiOpenInputBox(buf,cb_tgt_set,0,20,(void*)this);
 return true;
}
//#######################################################################################//
int txt_len(HDC hDC,char *str)
{
 SIZE size;
 GetTextExtentPoint32(hDC,str,(int)strlen(str),&size);
 return (int)size.cx;
}
//#######################################################################################//
int txt_hei(HDC hDC)
{
 TEXTMETRIC tm;
 GetTextMetrics(hDC,&tm);
 return tm.tmHeight;
}
//#######################################################################################//
void uap_planner_mfd::Update(HDC hDC)
{
 VESSEL *ves=oapiGetFocusInterface();
 get_tgt();
 
 int sp,n,h,line=0;
 char s[100];
 h=abs(txt_hei(hDC));
 
 sprintf(s,"UAP flight planner");n=txt_len(hDC,s);TextOutA(hDC,(scrx-n)/2,2,s,sizeof(char)*strlen(s));
 
 SetTextColor(hDC,RGB(255,255,255));
 sprintf(s,"Greetings");n=txt_len(hDC,s);TextOutA(hDC,(scrx-n)/2,2+1*h,s,sizeof(char)*strlen(s));
 sprintf(s,"Where do you want to end up today?");n=txt_len(hDC,s);TextOutA(hDC,(scrx-n)/2,2+2*h,s,sizeof(char)*strlen(s));
 
 SetTextColor(hDC,RGB(128,255,128));
 sp=scry/7;
 sprintf(s,"<- 2. Docked to ...");TextOutA(hDC,2,2*sp,s,sizeof(char)*strlen(s));
 sprintf(s,"<- 3. Landed at ...");TextOutA(hDC,2,3*sp,s,sizeof(char)*strlen(s));
 sprintf(s,"<- 4. In orbit of ...");TextOutA(hDC,2,4*sp,s,sizeof(char)*strlen(s));
 
 //SetTextColor(hDC,RGB(128,255,128));
 //sprintf(s,"In %s (%s)",ves->GetName(),ves->GetClassName());print(hDC,10,&line,s);
  /*
 if(mfd_seq==-1){
  SetTextColor(hDC,RGB(255,255,255));
  sprintf(s,"Greetings");print(hDC,10,&line,s);
  sprintf(s,"Where do you want to end up today?");print(hDC,10,&line,s);
 }else{
  if(seqs[mfd_seq].run){
   SetTextColor(hDC,RGB(255,0,0));
   sprintf(s,"Running sequence");print(hDC,10,&line,s);
   if(!uap_time_acc){SetTextColor(hDC,RGB((255*(sin(oapiGetSimTime()*4)+1)/2),(255*(cos(oapiGetSimTime()*4)+1)/2),0));sprintf(s,"Time accel control OFF");print(hDC,10,&line,s);SetTextColor(hDC,RGB(255,0,0));}
   sprintf(s,"Step %d of %d, %s",seqs[mfd_seq].cur,seqs[mfd_seq].cnt-1,seqs[mfd_seq].seq[seqs[mfd_seq].cur].ap->name);print(hDC,10,&line,s);
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
 */
}
//#######################################################################################//
int uap_planner_mfd::MsgProc(UINT msg,UINT mfd,WPARAM wparam,LPARAM lparam){switch(msg){case OAPI_MSG_MFD_OPENED:return(int)(new uap_planner_mfd(LOWORD(wparam),HIWORD(wparam),(VESSEL*)lparam));}return 0;}
//#######################################################################################//
