# -*- coding: utf-8 -*-
# @File   		: vtk_demo.py
# @Description	: render puncture needle
# @Date   		: 2024/02/20 17:33:43
# @Author	    : zxliao, zhixiangleo@163.com

import vtk
import numpy as np
import threading
import asyncio

# stl = "./puncture_robot_model/needle.stl"
stl = "./ur5_model/base.stl"

def simple_display(stl_file):
    colors = vtk.vtkNamedColors()

    sphere = vtk.vtkSphereSource()
    sphere.SetCenter(0,0,0)
    sphere.SetRadius(0.02)
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
    prpperty = stl_actor.GetProperty()
    # stl_actor.GetProperty().SetColor(colors.GetColor3d("slate_grey"))
    prpperty.SetColor(colors.GetColor3d("slate_grey"))
    # 设置漫反射，漫反射系数(Diffuse): 光线照射到物体材质上，经过漫反射后形成的光线强度, 越大时，物体偏亮
    stl_actor.GetProperty().SetDiffuse(1)
    # 设定反射色，光线照射到物体材质上，经过镜面反射后形成的光线强度
    stl_actor.GetProperty().SetSpecular(0.4)
    # 镜面指数(Specular Power): 取值范围是0---128，该值越小，表示材质越是粗糙，当点光源发射的光线照射到上面时，可以产生较大的亮点，
    # 该值越大，表示材质越是类似于镜面，光源照射到上面后会产生较小的亮点；
    stl_actor.GetProperty().SetSpecularPower(20)
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
    # renderer.AddActor(stl_actor)
    renderer.AddActor(sphere_actor)
    # renderer.AddActor(axes)
    renderer.SetBackground(colors.GetColor3d("linen"))

    # 创建渲染窗口
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    render_window.SetSize(800,800)

    # 创建渲染窗口交互器
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(render_window)
    # style = vtk.vtkInteractorStyleTrackballActor()# 通过改变actor来改变物体的位置和大小
    style = vtk.vtkInteractorStyleTrackballCamera()# 通过修改相机来改变观察角度和物体大小
    style.SetDefaultRenderer(renderer)
    interactor.SetInteractorStyle(style)

    # vtkBoxWidget()是一个正交六面体3D控件。
    # 这个3D小部件定义了一个区域，
    # 该区域由一个内表面角度为90度（正交面）的任意定向六面体表示。
    # 该对象创建了7个可在其上进行鼠标移动和操作的控制柄。
    # 此外，还显示了一个边界框轮廓，可以选择其“面”进行对象旋转或缩放。
    box_widget = vtk.vtkBoxWidget()
    box_widget.SetInteractor(interactor)
    box_widget.SetPlaceFactor(1.25)
    box_widget.GetOutlineProperty().SetColor(colors.GetColor3d('gold'))
    # 小部件的典型用法是使用StartInteractionEvent、InteractionEvent和EndInteractionEvent事件。
    # 在鼠标移动时调用InteractionEvent；
    # 另外两个事件分别称为“按下按钮”和“按下按钮”（左键或右键）。
    box_widget.SetProp3D(sphere_actor)
    box_widget.PlaceWidget()
    cb_box_widget = vtkMyCallBack()
    box_widget.AddObserver('InteractionEvent',cb_box_widget)
    box_widget.On()

    # 开始渲染循环
    render_window.Render()
    interactor.Initialize()
    interactor.Start()

class vtkMyCallBack(object):
    def __call__(self,caller,ev):
        tf = vtk.vtkTransform()
        widget = caller
        widget.GetTransform(tf)
        widget.GetProp3D().SetUserTransform(tf)

def numpy_to_image(numpy_array):
    """
    @brief Convert a numpy 2D or 3D array to a vtkImageData object
    @param numpy_array 2D or 3D numpy array containing image data
    @return vtkImageData with the numpy_array content
    """

    shape = numpy_array.shape
    if len(shape) < 2:
        raise Exception('numpy array must have dimensionality of at least 2')

    h, w = shape[0], shape[1]
    c = 1
    if len(shape) == 3:
        c = shape[2]

    # Reshape 2D image to 1D array suitable for conversion to a
    # vtkArray with numpy_support.numpy_to_vtk()
    linear_array = np.reshape(numpy_array, (w*h, c))
    vtk_array = vtk.util.numpy_support.numpy_to_vtk(linear_array)

    image = vtk.vtkImageData()
    image.SetDimensions(w, h, 1)
    image.AllocateScalars()
    image.GetPointData().GetScalars().DeepCopy(vtk_array)

    return image 

def display_cross_line():
    dicom_folder = "F:/0_project/prca/dicom/20240225/2024.02.25-144314-STD-1.3.12.2.1107.5.99.3/20240225/1.3.12.2.1107.5.1.7.120479.30000024022512255527200003523"
    # 读取 DICOM 数据
    reader = vtk.vtkDICOMImageReader()
    reader.SetDirectoryName(dicom_folder)
    reader.Update()
    # 获取 DICOM 数据信息
    extent = reader.GetDataExtent()
    spacing = reader.GetPixelSpacing()
    # 将 DICOM 数据转换为 VTK 图像数据
    image_data = reader.GetOutput()
    # 创建 VTK 渲染器
    renderer = vtk.vtkRenderer()
    # 创建 VTK 窗口
    render_window = vtk.vtkRenderWindow()
    render_window.SetSize(512, 512)
    render_window.AddRenderer(renderer)
    # 创建 VTK 交互器
    interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(render_window)
    # 创建 VTK 映射器和切片视图
    viewer = vtk.vtkImageViewer2()
    viewer.SetInputData(image_data)
    viewer.SetSliceOrientationToXY()
    viewer.SetupInteractor(interactor)

    # 创建十字叉的线
    crosshair_lines = vtk.vtkCellArray()
    crosshair_points = vtk.vtkPoints()
    crosshair_points.SetNumberOfPoints(4)

    # 十字叉在中心位置
    center = [extent[0] + (extent[1] - extent[0]) / 2,
            extent[2] + (extent[3] - extent[2]) / 2,
            extent[4] + (extent[5] - extent[4]) / 2]

    # 十字叉的长度
    crosshair_length = 50

    # 添加十字叉的线
    crosshair_points.InsertPoint(0, center[0], center[1] - crosshair_length, center[2])
    crosshair_points.InsertPoint(1, center[0], center[1] + crosshair_length, center[2])
    crosshair_points.InsertPoint(2, center[0] - crosshair_length, center[1], center[2])
    crosshair_points.InsertPoint(3, center[0] + crosshair_length, center[1], center[2])

    crosshair_lines.InsertNextCell(2)
    crosshair_lines.InsertCellPoint(0)
    crosshair_lines.InsertCellPoint(1)

    crosshair_lines.InsertNextCell(2)
    crosshair_lines.InsertCellPoint(2)
    crosshair_lines.InsertCellPoint(3)

    # 创建 VTK 线数据
    crosshair_polydata = vtk.vtkPolyData()
    crosshair_polydata.SetPoints(crosshair_points)
    crosshair_polydata.SetLines(crosshair_lines)

    # 创建 VTK 线映射器
    crosshair_mapper = vtk.vtkPolyDataMapper()
    crosshair_mapper.SetInputData(crosshair_polydata)

    # 创建 VTK 线 Actor
    crosshair_actor = vtk.vtkActor()
    crosshair_actor.SetMapper(crosshair_mapper)
    crosshair_actor.GetProperty().SetLineStipplePattern(0xF0F0)  # 设置虚线样式
    crosshair_actor.GetProperty().SetLineStippleRepeatFactor(2)    # 
    crosshair_actor.GetProperty().SetColor(1.0, 0.0, 0.0)  # 设置红色

    # 将十字叉 Actor 添加到渲染器中
    viewer.GetRenderer().AddActor(crosshair_actor)
    viewer.SetSlice(10)

    # 设置渲染器的背景色
    # renderer.SetBackground(0.1, 0.2, 0.4)
    viewer.GetRenderer().SetBackground(0.1, 0.2, 0.4)

    # 启动交互器
    viewer.Render()
    interactor.Initialize()
    interactor.Start()

def vtk_line():
    dicom_folder = "F:/0_project/prca/dicom/20240225/2024.02.25-144314-STD-1.3.12.2.1107.5.99.3/20240225/1.3.12.2.1107.5.1.7.120479.30000024022512255527200003523"
    # 读取 DICOM 数据
    reader = vtk.vtkDICOMImageReader()
    reader.SetDirectoryName(dicom_folder)
    reader.Update()

    viewer = vtk.vtkImageViewer2()
    viewer.SetInputConnection(reader.GetOutputPort())

    # 获取 viewer 的渲染器
    renderer = viewer.GetRenderer()

    # 创建一个 vtkLineWidget2
    line_widget = vtk.vtkLineWidget2()
    line_widget.CreateDefaultRepresentation()
    line_widget.GetLineRepresentation().SetPoint1DisplayPosition([10,100,0])
    line_widget.GetLineRepresentation().SetPoint1DisplayPosition([30,200,0])

    # 设置交互器
    interactor = vtk.vtkRenderWindowInteractor()
    viewer.SetupInteractor(interactor)

    # 将线添加到渲染器中
    renderer.AddActor(line_widget.GetRepresentation())

    # 渲染并启动交互器
    viewer.Render()
    interactor.Start()

async def render_window1():
    renderer1 = vtk.vtkRenderer()
    renderWindow1 = vtk.vtkRenderWindow()
    renderWindow1.AddRenderer(renderer1)
    renderWindowInteractor1 = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor1.SetRenderWindow(renderWindow1)

    cubeSource = vtk.vtkCubeSource()
    cubeMapper1 = vtk.vtkPolyDataMapper()
    cubeMapper1.SetInputConnection(cubeSource.GetOutputPort())
    cubeActor1 = vtk.vtkActor()
    cubeActor1.SetMapper(cubeMapper1)
    renderer1.AddActor(cubeActor1)

    renderWindow1.SetSize(300, 300)
    renderWindow1.SetWindowName("Window 1")
    renderWindow1.SetPosition(0, 0)
    
    renderWindowInteractor1.Initialize()
    await asyncio.sleep(0.1)  # 让出主线程控制权
    renderWindowInteractor1.Start()

async def render_window2():
    renderer2 = vtk.vtkRenderer()
    renderWindow2 = vtk.vtkRenderWindow()
    renderWindow2.AddRenderer(renderer2)
    renderWindowInteractor2 = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor2.SetRenderWindow(renderWindow2)

    cubeSource = vtk.vtkCubeSource()
    cubeMapper2 = vtk.vtkPolyDataMapper()
    cubeMapper2.SetInputConnection(cubeSource.GetOutputPort())
    cubeActor2 = vtk.vtkActor()
    cubeActor2.SetMapper(cubeMapper2)
    renderer2.AddActor(cubeActor2)

    renderWindow2.SetSize(300, 300)
    renderWindow2.SetWindowName("Window 2")
    renderWindow2.SetPosition(400, 0)

    renderWindowInteractor2.Initialize()
    await asyncio.sleep(0.1)  # 让出主线程控制权
    renderWindowInteractor2.Start()

async def vtk_thread():
    task1 = asyncio.create_task(render_window1())
    task2 = asyncio.create_task(render_window2())
    await asyncio.gather(task1, task2)



if __name__ == '__main__':
    # simple_display(stl)
    # display_cross_line()
    # vtk_line()
    asyncio.run(vtk_thread())
