import vtk

dcm_path = "F:/0_project/prca/dicom/20240225/2024.02.25-144314-STD-1.3.12.2.1107.5.99.3/20240225/1.3.12.2.1107.5.1.7.120479.30000024022512255527200003523"

def main():
    # 读取DICOM文件夹
    reader = vtk.vtkDICOMImageReader()
    reader.SetDirectoryName(dcm_path)
    reader.Update()

    # 创建四个渲染窗口以显示不同的视图
    renderWindows = [vtk.vtkRenderWindow() for _ in range(4)]
    renderers = [vtk.vtkRenderer() for _ in range(4)]

    # 显示横断面、冠状面、矢状面
    for i, orientation in enumerate(["Axial", "Coronal", "Sagittal"]):
        reslice = vtk.vtkImageReslice()
        reslice.SetInputConnection(reader.GetOutputPort())
        reslice.SetOutputDimensionality(2)
        if orientation == "Axial":
            reslice.SetResliceAxesDirectionCosines(1, 0, 0, 0, 1, 0, 0, 0, 1)
        elif orientation == "Coronal":
            reslice.SetResliceAxesDirectionCosines(1, 0, 0, 0, 0, 1, 0, -1, 0)
        elif orientation == "Sagittal":
            reslice.SetResliceAxesDirectionCosines(0, 0, 1, 0, 1, 0, -1, 0, 0)
        reslice.Update()

        imageActor = vtk.vtkImageActor()
        imageActor.SetInputData(reslice.GetOutput())

        renderers[i].AddActor(imageActor)
        renderers[i].ResetCamera()

    for i, window in enumerate(renderWindows):
        window.AddRenderer(renderers[i])
        window.SetPosition(400 * (i % 2), 400 * (i // 2))  # 窗口位置
        window.SetSize(400, 400)  # 窗口大小

    # 3D重建
    volumeMapper = vtk.vtkSmartVolumeMapper()
    volumeMapper.SetInputConnection(reader.GetOutputPort())

    volumeProperty = vtk.vtkVolumeProperty()
    volumeProperty.ShadeOn()
    volumeProperty.SetInterpolationTypeToLinear()

    volume = vtk.vtkVolume()
    volume.SetMapper(volumeMapper)
    volume.SetProperty(volumeProperty)

    renderers[3].AddVolume(volume)
    renderers[3].ResetCamera()

    # 启动渲染循环
    for window in renderWindows:
        window.Render()
        iren = vtk.vtkRenderWindowInteractor()
        iren.SetRenderWindow(window)
        iren.Start()

    # vtk.vtkRenderWindowInteractor().Start()

if __name__ == "__main__":
    main()