# -*- coding: utf-8 -*-
# @File   		: vtk_demo.py
# @Description	: do something
# @Date   		: 2024/02/20 17:33:43
# @Author	    : zxliao, zhixiangleo@163.com

import vtk

# stl = "./puncture_robot_model/needle.stl"
stl = "./ur5_model/base.stl"

def simple_display(stl_file):
    colors = vtk.vtkNamedColors()

    sphere = vtk.vtkSphereSource()
    sphere.SetCenter(0,0,0)
    sphere.SetRadius(5)
    sphere.SetThetaResolution(64)
    sphere.SetPhiResolution(64)
    sphere_mapper = vtk.vtkPolyDataMapper()
    sphere_mapper.SetInputConnection(sphere.GetOutputPort())
    sphere_actor = vtk.vtkActor()
    sphere_actor.SetMapper(sphere_mapper)

    # 创建STL文件读取器
    stl_reader = vtk.vtkSTLReader()
    stl_reader.SetFileName(stl_file)

    # 创建映射器来映射STL数据
    stl_mapper = vtk.vtkPolyDataMapper()
    stl_mapper.SetInputConnection(stl_reader.GetOutputPort())

    # 使用映射器创建一个actor
    stl_actor = vtk.vtkActor()
    stl_actor.SetMapper(stl_mapper)
    stl_actor.GetProperty().SetColor(colors.GetColor3d("slate_grey"))
    print(stl_actor.GetOrigin())
    print(stl_actor.GetOrientation())

    transform = vtk.vtkTransform()
    transform.Translate(10.0, 0.0, 0.0)

    axes = vtk.vtkAxesActor()
    axes.SetUserTransform(transform)
    axes.AxisLabelsOff()
    axes.SetTotalLength(10,10,10)

    # 创建渲染器并添加actor
    renderer = vtk.vtkRenderer()
    renderer.AddActor(stl_actor)
    # renderer.AddActor(sphere_actor)
    # renderer.AddActor(axes)
    renderer.SetBackground(colors.GetColor3d("linen"))

    # 创建渲染窗口
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    render_window.SetSize(800,800)

    # 创建渲染窗口交互器
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(render_window)
    style = vtk.vtkInteractorStyleTrackballCamera()
    style.SetDefaultRenderer(renderer)
    interactor.SetInteractorStyle(style)

    # 开始渲染循环
    render_window.Render()
    interactor.Initialize()
    interactor.Start()


if __name__ == '__main__':
    simple_display(stl)
