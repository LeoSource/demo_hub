# -*- coding: utf-8 -*-
# @File   		: vtk_demo.py
# @Description	: render puncture needle
# @Date   		: 2024/02/20 17:33:43
# @Author	    : zxliao, zhixiangleo@163.com

import vtk
import numpy as np

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

if __name__ == '__main__':
    simple_display(stl)
