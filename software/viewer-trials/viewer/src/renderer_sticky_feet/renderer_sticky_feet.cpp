#include "renderer_sticky_feet.hpp"
#include "FootStepPlanListener.hpp" // need parent renderer struct which contains a FootStepPlanListener... circular dependency.
#include "plan_approval_gui_utils.hpp"
#include "plan_execution_gui_utils.hpp"
#include <glib.h>
#include <bot_vis/gl_util.h>
#include <bot_core/fasttrig.h>
#include "lcmtypes/drc_utime_t.h"
#include "lcmtypes/drc_footstep_plan_t.h"

#define RENDERER_NAME "Footstep Plans"
#define PARAM_SHOW_DETAILS "Show Step Details"
#define PARAM_CLEAR_FOOTSTEP_PLAN "Clear Footsteps"
using namespace std;
using namespace boost;
using namespace renderer_sticky_feet;
using namespace renderer_sticky_feet_gui_utils;

static void
_renderer_free (BotRenderer *super)
{
  RendererStickyFeet *self = (RendererStickyFeet*) super->user;
  delete self->perceptionData;
  free(self);
}
//================================= Drawing

static void 
draw_state(BotViewer *viewer, BotRenderer *super, uint i){
  RendererStickyFeet *self = (RendererStickyFeet*) super->user;

  float c_blue[3] = {0.3,0.3,0.6}; // light blue
  float c_grey[3] = {0.3,0.3,0.3}; // grey

  float c_green[3] = {0.3,0.5,0.3};  // green for right sticky feet
  float c_yellow[3] = {0.5,0.5,0.3}; //yellow for left sticky feet

  if (!self->footStepPlanListener->_allow_execution){
    c_green[0] = 0.7;
    c_green[1] = 0.3;
    c_green[2] = 0.3;
    c_yellow[0] = 0.3;
    c_yellow[1] = 0.3;
    c_yellow[2] = 0.7;
    // if non executable steps, color red
  }

  float alpha_contact = 0.4;
  float alpha_apex = 0.15;
  
//  self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->show_bbox(self->visualize_bbox);
//  self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->enable_link_selection(self->ht_auto_adjust_enabled);
string no_selection =  " ";
 if((*self->selection)== self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_unique_name){   
   if((*self->marker_selection)==" ")
    self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_link((*self->selection));
   else
    self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_marker((*self->marker_selection));
 }
 else{ 
   self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_link(no_selection);
   self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_marker(no_selection);
 }
   
 if((*self->marker_selection)!=" "){ 
    if(self->footStepPlanListener->is_motion_copy(i))
       self->footStepPlanListener->_gl_in_motion_copy->highlight_marker((*self->marker_selection));
 }
 else{ 
   self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->highlight_marker(no_selection);
 }
 
  float alpha;
  if (self->footStepPlanListener->_planned_stickyfeet_info_list[i].is_in_contact) {
    alpha = alpha_contact;
  } else {
    alpha = alpha_apex;
  }
  if(!self->footStepPlanListener->_planned_stickyfeet_info_list[i].is_fixed){
    if(self->footStepPlanListener->_planned_stickyfeet_info_list[i].foot_type == FootStepPlanListener::RIGHT)
      self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->draw_body (c_green,alpha); 
    else
      self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->draw_body (c_yellow,alpha);  
  }
  else {
    self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->draw_body (c_grey,alpha); 
  }   
  
    

 if(self->footStepPlanListener->is_motion_copy(i))
   self->footStepPlanListener->_gl_in_motion_copy->draw_body (c_blue,alpha);

  //self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->draw_whole_body_bbox();
}

static void 
_renderer_draw (BotViewer *viewer, BotRenderer *super){
  RendererStickyFeet *self = (RendererStickyFeet*) super->user;

  glEnable(GL_DEPTH_TEST);

  //-draw 
  //glPointSize(5.0f);
  //glBegin(GL_POINTS);
  glEnable(GL_LIGHTING);
  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_BLEND);
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

  // if((self->ht_auto_adjust_enabled)&&(self->clicked)){
  //   glLineWidth (3.0);
  //   glPushMatrix();
  //   glBegin(GL_LINES);
  //   glVertex3f(self->ray_start[0], self->ray_start[1],self->ray_start[2]); // object coord
  //   glVertex3f(self->ray_end[0], self->ray_end[1],self->ray_end[2]);
  //   glEnd();
  //   glPopMatrix();
  // }

  float label_num = 0;
  bool has_apex = false;
  for(uint i = 0; i < self->footStepPlanListener->_gl_planned_stickyfeet_list.size(); i++){ 
    //cout << "i:"<<i<< endl;
    double pos[3];
    pos[0] = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body.p[0]; 
    pos[1] = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body.p[1]; 
    pos[2] = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body.p[2]+0.03;  

    if (i > 1) { // don't label the two steps where the robot's feet already are
      std::stringstream oss;
      if (!self->footStepPlanListener->_planned_stickyfeet_info_list[i].is_in_contact) {
        label_num += 0.5;
        has_apex = true;
      } else {
        if (has_apex) {
          label_num += 0.5;
        } else {
          label_num += 1;
        }
        has_apex = false;
      }
      // float label_num = (i - 1.0) / 2.0;
      KDL::Frame T_worldframe_prev_foot = self->footStepPlanListener->_gl_planned_stickyfeet_list[i-1]->_T_world_body;
      KDL::Frame T_worldframe_curr_foot = self->footStepPlanListener->_gl_planned_stickyfeet_list[i]->_T_world_body;
      KDL::Frame T_prev_to_curr = T_worldframe_prev_foot.Inverse() * T_worldframe_curr_foot;

      double camera_dist_sq = pow(T_worldframe_curr_foot.p[0] - self->ray_start[0], 2) + pow(T_worldframe_curr_foot.p[1] - self->ray_start[1], 2) + pow(T_worldframe_curr_foot.p[2] - self->ray_start[2], 2);

      char buff[100];
      if (self->show_detailed_info && camera_dist_sq < 8) {
        snprintf(buff, 100, "%.0f \ndx %+.2f\ndy %+.2f\ndz %+.2f", label_num, T_prev_to_curr.p[0], T_prev_to_curr.p[1], T_prev_to_curr.p[2]);
        glColor4f(1,1,1,1);
        bot_gl_draw_text(pos, GLUT_BITMAP_HELVETICA_12, buff,BOT_GL_DRAW_TEXT_JUSTIFY_LEFT | BOT_GL_DRAW_TEXT_ANCHOR_BOTTOM | BOT_GL_DRAW_TEXT_DROP_SHADOW);
      } else {
        snprintf(buff, 100, "%.0f", label_num);
        glColor4f(0,0,0,1);
        bot_gl_draw_text(pos, GLUT_BITMAP_HELVETICA_18, buff,BOT_GL_DRAW_TEXT_JUSTIFY_LEFT);
      }
    }
    
    draw_state(viewer,super,i);
  }
   
  if(!self->footStepPlanListener->_bdi_footstep_mode){ 
    if(!self->footStepPlanListener->_last_plan_approved_or_executed){
      if((self->footStepPlanListener->_gl_planned_stickyfeet_list.size()>0)&&(self->plan_approval_dock==NULL))
          spawn_plan_approval_dock(self);
    } 
  }else{
    if(!self->footStepPlanListener->_last_plan_approved_or_executed){
      if((self->footStepPlanListener->_gl_planned_stickyfeet_list.size()>0)&&(self->plan_execute_button==NULL)){
        if(self->plan_execution_dock!=NULL){
         gtk_widget_destroy (self->plan_execution_dock);
         self->plan_execution_dock= NULL;
        }
        spawn_plan_execution_dock(self);
      }
    }
  }   

}

//========================= Event Handling ================

static double pick_query (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3])
{
  RendererStickyFeet *self = (RendererStickyFeet*) ehandler->user;

  //fprintf(stderr, "RobotStateRenderer Pick Query Active\n");
  Eigen::Vector3f from,to;
  from << ray_start[0], ray_start[1], ray_start[2];

  Eigen::Vector3f plane_normal,plane_pt;
  plane_normal << 0,0,1;
  if(ray_start[2]<0)
      plane_pt << 0,0,10;
  else
      plane_pt << 0,0,-10;
  double lambda1 = ray_dir[0] * plane_normal[0]+
                   ray_dir[1] * plane_normal[1] +
                   ray_dir[2] * plane_normal[2];
   // check for degenerate case where ray is (more or less) parallel to plane
    if (fabs (lambda1) < 1e-9) return -1.0;

   double lambda2 = (plane_pt[0] - ray_start[0]) * plane_normal[0] +
       (plane_pt[1] - ray_start[1]) * plane_normal[1] +
       (plane_pt[2] - ray_start[2]) * plane_normal[2];
   double t = lambda2 / lambda1;// =1;
  
  to << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];

  self->ray_start = from;
  self->ray_end = to;
  self->ray_hit_t = t;
  self->ray_hit_drag = to;
  self->ray_hit = to; 
  
  double shortest_distance = get_shortest_distance_between_stickyfeet_and_markers(self,from,to);
  
  return shortest_distance;
}

static int mouse_press (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], const double ray_dir[3], const GdkEventButton *event)
{
  RendererStickyFeet *self = (RendererStickyFeet*) ehandler->user;
  if((ehandler->picking==0)){
    //fprintf(stderr, "Ehandler Not active\n");
   (*self->selection)  = " ";
    return 0;
  }

 // fprintf(stderr, "RobotPlanRenderer Ehandler Activated\n");
  self->clicked = 1;
  //fprintf(stderr, "Mouse Press : %f,%f\n",ray_start[0], ray_start[1]);

  if((self->selected_planned_footstep_index>=0)&&(self->selected_planned_footstep_index < self->footStepPlanListener->_gl_planned_stickyfeet_list.size()))
  { 
    collision::Collision_Object * intersected_object = NULL;
    self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->_collision_detector->ray_test( self->ray_start, self->ray_end, intersected_object );
    if( intersected_object != NULL ){
        //cout << self->selected_planned_footstep_index << endl;  
        //std::cout << "prev selection :" << (*self->selection)  <<  std::endl;
      (*self->selection)  = self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->_unique_name;
       std::cout << "intersected sticky foot:" << (*self->selection) <<  std::endl;
       // self->footStepPlanListener->_gl_planned_stickyfeet_list[self->selected_planned_footstep_index]->highlight_link((*self->selection));
     }
  }
 
  if((((*self->selection)  != " ") || ((*self->marker_selection)  != " ")) &&
     (event->button==1) &&
     (self->selected_planned_footstep_index>=0) &&
     (self->selected_planned_footstep_index < self->footStepPlanListener->_gl_planned_stickyfeet_list.size()) &&
     (event->type==GDK_2BUTTON_PRESS)
    )
  {
    std::cout << "dbl clk on  marker:" << (*self->marker_selection) <<  std::endl;
    //if((*self->marker_selection)  == " ")// dbl clk on link then toogle
    {     
       bool toggle=true;
       if (self->footStepPlanListener->is_motion_copy(self->selected_planned_footstep_index))
           toggle = !self->footStepPlanListener->_gl_in_motion_copy->is_bodypose_adjustment_enabled();
        self->footStepPlanListener->create_sticky_foot_local_copy(self->selected_planned_footstep_index);
        self->footStepPlanListener->_gl_in_motion_copy->enable_bodypose_adjustment(toggle);   
    }
  
    bot_viewer_request_redraw(self->viewer);
    std::cout << "RendererStickyFeet: Event is consumed" <<  std::endl;
    return 1;// consumed if pop up comes up.
  }
  else if(((*self->marker_selection)  != " "))
  {
    self->dragging = 1;

    KDL::Frame T_world_object;
    T_world_object = self->footStepPlanListener->_gl_in_motion_copy->_T_world_body; 
    self->marker_offset_on_press << self->ray_hit[0]-T_world_object.p[0],self->ray_hit[1]-T_world_object.p[1],self->ray_hit[2]-T_world_object.p[2]; 
    std::cout << "RendererStickyFeet: Event is consumed" <<  std::endl;
    return 1;// consumed
  }
  
  
  bot_viewer_request_redraw(self->viewer);

  return 0;
}


static int 
mouse_release (BotViewer *viewer, BotEventHandler *ehandler, const double ray_start[3], 
    const double ray_dir[3], const GdkEventButton *event)
{
  RendererStickyFeet *self = (RendererStickyFeet*) ehandler->user;
  self->clicked = 0;
  if((ehandler->picking==0)){
    //fprintf(stderr, "Ehandler Not active\n");
    return 0;
  }
  if (self->dragging) {
    self->dragging = 0;
  }
  if (ehandler->picking==1)
    ehandler->picking=0; //release picking(IMPORTANT)
  bot_viewer_request_redraw(self->viewer);
  return 0;
}

// ----------------------------------------------------------------------------
static int mouse_scroll (BotViewer *viewer, BotEventHandler *ehandler,  const double ray_start[3], const double ray_dir[3],   const GdkEventScroll *event)
{
  RendererStickyFeet *self = (RendererStickyFeet*) ehandler->user;

  Eigen::Vector3f from;
  from << ray_start[0], ray_start[1], ray_start[2];
  self->ray_start = from;
  return 0;
}

static int mouse_motion (BotViewer *viewer, BotEventHandler *ehandler,  const double ray_start[3], const double ray_dir[3],   const GdkEventMotion *event)
{
  RendererStickyFeet *self = (RendererStickyFeet*) ehandler->user;

  Eigen::Vector3f from;
  from << ray_start[0], ray_start[1], ray_start[2];
  self->ray_start = from;
  
  if((!self->dragging)||(ehandler->picking==0)){
    return 0;
  }
  
  if((*self->marker_selection)  != " "){
    double t = self->ray_hit_t;
    self->ray_hit_drag << ray_start[0]+t*ray_dir[0], ray_start[1]+t*ray_dir[1], ray_start[2]+t*ray_dir[2];
    set_object_desired_state_on_marker_motion(self);
  }
  bot_viewer_request_redraw(self->viewer);
  return 1;
}


static void onRobotUtime (const lcm_recv_buf_t * buf, const char *channel, 
                               const drc_utime_t *msg, void *user){
  RendererStickyFeet *self = (RendererStickyFeet*) user;
  self->robot_utime = msg->utime;
}

static void onPlanExecuteEvent (const lcm_recv_buf_t * buf, const char *channel, 
                               const drc_utime_t *msg, void *user)
{
    RendererStickyFeet *self = (RendererStickyFeet*) user;
    if((!self->footStepPlanListener->_last_plan_approved_or_executed)&&(self->plan_execute_button!=NULL))
    {
      gtk_widget_destroy (self->plan_execute_button);
      self->plan_execute_button= NULL;
      self->footStepPlanListener->_last_plan_approved_or_executed=true;
    }
}


static void on_param_widget_changed(BotGtkParamWidget *pw, const char *name, void *user)
{
  RendererStickyFeet *self = (RendererStickyFeet*) user;
  if (!strcmp(name, PARAM_SHOW_DETAILS)) {
    self->show_detailed_info = bot_gtk_param_widget_get_bool(pw, PARAM_SHOW_DETAILS);
  }
  else if(!strcmp(name, PARAM_CLEAR_FOOTSTEP_PLAN))
  {
    self->footStepPlanListener->_gl_planned_stickyfeet_list.clear();
  }

  bot_viewer_request_redraw(self->viewer);
  
}

void 
setup_renderer_sticky_feet(BotViewer *viewer, int render_priority, lcm_t *lcm, BotParam * param,
    BotFrames * frames, int operation_mode)
{
    RendererStickyFeet *self = (RendererStickyFeet*) calloc (1, sizeof (RendererStickyFeet));
    self->lcm = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcm));
    
    self->perceptionData = new PerceptionData();
    self->perceptionData->mBotWrapper.reset(new maps::BotWrapper(lcm,param,frames));
    self->perceptionData->mViewClient.setBotWrapper(self->perceptionData->mBotWrapper);
    self->perceptionData->mViewClient.start();
      
    
    // For message compression testing (toby's stuff
    self->footStepPlanListener = boost::shared_ptr<FootStepPlanListener>(new FootStepPlanListener(self->lcm,viewer,operation_mode));
    BotRenderer *renderer = &self->renderer;

    renderer->draw = _renderer_draw;
    renderer->destroy = _renderer_free;

    renderer->widget = bot_gtk_param_widget_new();
    renderer->name = (char *) RENDERER_NAME;
    if (operation_mode == 1){
      renderer->name =(char *) "Footstep Committed";
    }else if (operation_mode == 2){
      renderer->name =(char *) "Footstep MIT Frame";      
    }
    renderer->user = self;
    renderer->enabled = 1;

    self->viewer = viewer;
    

    self->pw = BOT_GTK_PARAM_WIDGET(renderer->widget);
    
    // C-style subscribe:
    drc_utime_t_subscribe(self->lcm->getUnderlyingLCM(),"ROBOT_UTIME",onRobotUtime,self); 
    drc_utime_t_subscribe(self->lcm->getUnderlyingLCM(),"FOOTSTEP_PLAN_EXECUTE_EVENT",onPlanExecuteEvent,self);

    bot_gtk_param_widget_add_buttons(self->pw, PARAM_CLEAR_FOOTSTEP_PLAN, NULL);
    bot_gtk_param_widget_add_booleans(self->pw, BOT_GTK_PARAM_WIDGET_CHECKBOX, PARAM_SHOW_DETAILS, 0, NULL);
//    bot_gtk_param_widget_add_buttons(self->pw, PARAM_START_PLAN, NULL);
//    bot_gtk_param_widget_add_buttons(self->pw, PARAM_SEND_COMMITTED_PLAN, NULL);

    self->show_detailed_info = 0;
    bot_gtk_param_widget_set_bool(self->pw, PARAM_SHOW_DETAILS, self->show_detailed_info);
  	
  	g_signal_connect(G_OBJECT(self->pw), "changed", G_CALLBACK(on_param_widget_changed), self);
    self->clicked = 0;	
    self->dragging = 0;	
  	self->selection = new std::string(" ");
    self->marker_selection = new std::string(" ");
    self->selected_planned_footstep_index = -1;
    //bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
    bot_viewer_add_renderer_on_side(viewer,&self->renderer, render_priority, 0);
        
    BotEventHandler *ehandler = &self->ehandler;
    ehandler->name = (char*) RENDERER_NAME;
    ehandler->enabled = 1;
    ehandler->pick_query = pick_query;
    ehandler->hover_query = NULL;
    ehandler->mouse_press = mouse_press;
    ehandler->mouse_release = mouse_release;
    ehandler->mouse_motion = mouse_motion;
    ehandler->mouse_scroll = mouse_scroll;
    ehandler->user = self;
    
    if (operation_mode ==1){
      ehandler->name =(char *) "Footstep Committed";
    }else if (operation_mode == 2){
      ehandler->name =(char *) "Footstep MIT Frame";      
    }
    

    bot_viewer_add_event_handler(viewer, &self->ehandler, render_priority);
    
}
