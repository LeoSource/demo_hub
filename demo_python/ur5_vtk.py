# -*- coding: utf-8 -*-
# @File   		: ur5_vtk.py
# @Description	: do something
# @Date   		: 2024/02/21 16:13:32
# @Author	    : zxliao, zhixiangleo@163.com

import vtk

filenames = ['./ur5_model/base.stl',
             './ur5_model/forearm.stl',
             './ur5_model/shoulder.stl',
             './ur5_model/upperarm.stl',
             './ur5_model/wrist1.stl',
             './ur5_model/wrist2.stl',
             './ur5_model/wrist3.stl']

ren_win = vtk.vtkRenderWindow()
assemblys = list()
actors = list()

class MyInteractor(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self,parent=None) -> None:
        self.AddObserver("CharEvent",self.OnCharEvent)
        self.AddObserver("KeyPressEvent",self.OnKeyPressEvent)

    def OnCharEvent(self,obj,event):
        pass

    def OnKeyPressEvent(self,obj,event):
        pass

def load_stl_to_actor(filename):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(filename)
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor

def create_coordinate():
    axes = vtk.vtkAxesActor()
    axes.SetTotalLength(100,100,100)
    axes.SetShaftType(0)
    axes.SetCylinderRadius(0.02)
    axes.GetXAxisCaptionActor2D().SetWidth(0.03)
    axes.GetYAxisCaptionActor2D().SetWidth(0.03)
    axes.GetZAxisCaptionActor2D().SetWidth(0.03)
    return axes

def slider_callback1(obj,event):
    slider = obj.GetRepresentation()
    pos = slider.GetValue()
    assemblys[1].SetOrientation(0,0,pos)
    ren_win.Render()

def slider_callback2(obj,event):
    slider = obj.GetRepresentation()
    pos = slider.GetValue()
    assemblys[2].SetOrientation(0,0,pos)
    ren_win.Render()

def slider_callback3(obj,event):
    slider = obj.GetRepresentation()
    pos = slider.GetValue()
    assemblys[3].SetOrientation(0,0,pos)
    ren_win.Render()

def slider_callback4(obj,event):
    slider = obj.GetRepresentation()
    pos = slider.GetValue()
    assemblys[4].SetOrientation(0,0,pos)
    ren_win.Render()

def slider_callback5(obj,event):
    slider = obj.GetRepresentation()
    pos = slider.GetValue()
    assemblys[5].SetOrientation(0,0,pos)
    ren_win.Render()

def slider_callback6(obj,event):
    slider = obj.GetRepresentation()
    pos = slider.GetValue()
    assemblys[6].SetOrientation(0,0,pos)
    ren_win.Render()

def create_slide(iren,position):
    slider_widget = config_slider(vtk.vtkSliderRepresentation2D(),position)
    slider_widget.SetInteractor(iren)
    slider_widget.EnabledOn()
    return slider_widget

def config_slider(slider,yaxis):
    slider.SetMinimumValue(0)
    slider.SetMaximumValue(360)
    slider.SetValue(0)
    slider.GetSliderProperty().SetColor(1,0,0)
    slider.GetSelectedProperty().SetColor(0, 0, 1)
    slider.GetTubeProperty().SetColor(1, 1, 0)
    slider.GetCapProperty().SetColor(0, 1, 1)
    slider.GetPoint1Coordinate().SetCoordinateSystemToDisplay()
    slider.GetPoint1Coordinate().SetValue(50, yaxis)
    slider.GetPoint2Coordinate().SetCoordinateSystemToDisplay()
    slider.GetPoint2Coordinate().SetValue(400, yaxis)
    slider.SetSliderLength(0.02)
    slider.SetSliderWidth(0.02)
    slider.SetTubeWidth(0.005)
    slider.SetEndCapWidth(0.03)
    slider.ShowSliderLabelOn()
    slider.SetLabelFormat("%.1f")

    slider_widget = vtk.vtkSliderWidget()
    slider_widget.SetRepresentation(slider)
    slider_widget.SetAnimationModeToAnimate()

    return slider_widget


def create_ground():
    pass

def create_scene():
    ren = vtk.vtkRenderer()
    ren_win.AddRenderer(ren)
    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(ren_win)
    style = MyInteractor()
    style.SetDefaultRenderer(ren)
    iren.SetInteractorStyle(style)

    for idx,file in enumerate(filenames):
        actors.append(load_stl_to_actor(file))
        r = vtk.vtkMath.Random(.4,1.0)
        g = vtk.vtkMath.Random(.4,1.0)
        b = vtk.vtkMath.Random(.4,1.0)

        actors[idx].GetProperty().SetDiffuseColor(r,g,b)
        actors[idx].GetProperty().SetDiffuse(.8)
        actors[idx].GetProperty().SetSpecular(.5)
        actors[idx].GetProperty().SetSpecularColor(1,1,1)
        actors[idx].GetProperty().SetSpecularPower(30.0)

        tmp_assembly = vtk.vtkAssembly()
        assemblys.append(tmp_assembly)
        assemblys[idx].AddPart(actors[idx])
        assemblys[idx].SetPosition(0.01,0,0)
        if idx>0:
            assemblys[idx-1].AddPart(tmp_assembly)

    assemblys[0].SetOrigin(0,0,0)
    assemblys[1].SetOrigin(0,0,0)
    assemblys[2].SetOrigin(0,0,0)
    assemblys[3].SetOrigin(0,0,0)
    assemblys[4].SetOrigin(0,0,0)
    assemblys[5].SetOrigin(0,0,0)
    assemblys[6].SetOrigin(0,0,0)

    assm = vtk.vtkAssembly()
    assm.SetPosition(0,0,0)

    ren.AddActor(assemblys[0])
    # ren.AddActor(create_coordinate())
    # ren.AddActor(create_ground())

    slider1 = create_slide(iren,40)
    slider2 = create_slide(iren,80)
    slider3 = create_slide(iren,120)
    slider4 = create_slide(iren,160)
    slider5 = create_slide(iren,200)
    slider6 = create_slide(iren,240)

    slider1.AddObserver("InteractionEvent",slider_callback1)
    slider2.AddObserver("InteractionEvent",slider_callback2)
    slider3.AddObserver("InteractionEvent",slider_callback3)
    slider4.AddObserver("InteractionEvent",slider_callback4)
    slider5.AddObserver("InteractionEvent",slider_callback5)
    slider6.AddObserver("InteractionEvent",slider_callback6)

    ren.SetBackground(.2,.2,.2)
    ren_win.SetSize(600,600)

    # camera = vtk.vtkCamera()
    # camera.SetFocalPoint(300,0,0)
    # camera.SetPosition(300,-400,350)
    # camera.ComputeViewPlaneNormal()
    # camera.SetViewUp(0,1,0)
    # camera.Zoom(.4)
    # ren.SetActiveCamera(camera)

    iren.Initialize()
    iren.Start()
    


if __name__=='__main__':
    create_scene()