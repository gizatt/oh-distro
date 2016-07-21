import valkyriedriver
import valkyriedriverpanel
import exampletaskpanel
import tableboxdemo
import tableplanningdemo
import manualwalkingdemo
import stairsdemo
import trajectorytrackingtest

from director import tasklaunchpanel
from director import applogic
from director import teleoppanel

import tablemapping
import manualwalkingdemo
import calisthenicsdemo


def startup(robotSystem, globalsDict=None):
    rs = robotSystem

    valkyrieDriver = valkyriedriver.ValkyrieDriver(rs.ikPlanner, rs.handFactory)
    valkyrieDriverPanel = valkyriedriverpanel.init(valkyrieDriver)

    atlasPanelAction = applogic.getToolBarActions()['ActionAtlasDriverPanel']
    applogic.getMainWindow().panelToolBar().removeAction(atlasPanelAction)

    # add a new task panel
    #exampleTaskPanel = exampletaskpanel.ExampleTaskPanel(robotSystem)
    #tasklaunchpanel.panel.addTaskPanel('Example Task', exampleTaskPanel.widget)

    tableboxDemo = tableboxdemo.TableboxDemo(rs.robotStateModel, rs.playbackRobotModel,
                    rs.ikPlanner, rs.manipPlanner, rs.footstepsDriver, rs.lHandDriver, rs.rHandDriver,
                    rs.view, rs.robotStateJointController)
    tableboxTaskPanel = tableboxdemo.TableboxTaskPanel(tableboxDemo)
    tasklaunchpanel.panel.addTaskPanel('Tablebox', tableboxTaskPanel.widget)

    tableplanningDemo = tableplanningdemo.TableplanningDemo(rs.robotStateModel, rs.playbackRobotModel,
                    rs.ikPlanner, rs.manipPlanner, rs.footstepsDriver, rs.lHandDriver, rs.rHandDriver,
                    rs.view, rs.robotStateJointController, rs.teleopRobotModel, rs.teleopJointController, rs.footstepsDriver, valkyrieDriver)
    tableplanningTaskPanel = tableplanningdemo.TableplanningTaskPanel(tableplanningDemo)
    tasklaunchpanel.panel.addTaskPanel('Table Planning', tableplanningTaskPanel.widget)
    
    manualWalkingDemo = manualwalkingdemo.ManualWalkingDemo(rs.robotStateModel,
                    rs.footstepsDriver, rs.robotStateJointController, rs.ikPlanner)
    manualWalkingTaskPanel = manualwalkingdemo.ManualWalkingTaskPanel(manualWalkingDemo)
    tasklaunchpanel.panel.addTaskPanel('Manual Walking', manualWalkingTaskPanel.widget)

    stairsDemo = stairsdemo.StairsDemo(rs.robotStateModel, rs.footstepsDriver, rs.robotStateJointController, rs.ikPlanner, rs.manipPlanner)
    stairsTaskPanel = stairsdemo.StairsTaskPanel(stairsDemo)

    tasklaunchpanel.panel.addTaskPanel('Stairs', stairsTaskPanel.widget)

    tableMapping = tablemapping.TableMapping(rs.robotStateModel, rs.manipPlanner, rs.view,  rs.ikPlanner, rs.robotStateJointController)
    tableMappingTaskPanel = tablemapping.TableTaskPanel(tableMapping)
    tasklaunchpanel.panel.addTaskPanel("Table Mapping", tableMappingTaskPanel.widget)
    
    trajectoryTrackingTest = trajectorytrackingtest.TrajectoryTrackingTest(rs.ikPlanner, rs.manipPlanner, rs.robotStateJointController,
                    rs.footstepsDriver, rs.robotStateModel)
    trackingTestPanel = trajectorytrackingtest.TrackingTestPanel(trajectoryTrackingTest)
    tasklaunchpanel.panel.addTaskPanel('Tracking Test', trackingTestPanel.widget)

    calisthenicsDemo = calisthenicsdemo.CalisthenicsDemo(rs.robotStateModel,
                    rs.footstepsDriver, rs.robotStateJointController, rs.ikPlanner, rs.manipPlanner)
    calisthenicsTaskPanel = calisthenicsdemo.CalisthenicsTaskPanel(calisthenicsDemo)
    tasklaunchpanel.panel.addTaskPanel('Calisthenics', calisthenicsTaskPanel.widget)


    if globalsDict is not None:
        globalsDict['valkyrieDriver'] = valkyrieDriver
        globalsDict['valkyrieDriverPanel'] = valkyrieDriverPanel

        globalsDict['tableboxDemo'] = tableboxDemo
        globalsDict['tableplanningDemo'] = tableplanningDemo
        globalsDict['manualWalkingDemo'] = manualWalkingDemo
        globalsDict['manualWalkingTaskPanel'] = manualWalkingTaskPanel

        globalsDict['stairsDemo'] = stairsDemo
        globalsDict['stairsTaskPanel'] = stairsTaskPanel

        globalsDict['tableMappingTaskPanel'] = tableMappingTaskPanel

        globalsDict['calisthenicsDemo'] = calisthenicsDemo
