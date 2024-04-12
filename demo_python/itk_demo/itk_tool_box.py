import SimpleITK as sitk

class SimpleITKTool():
    
    def img_equalization(self,img):
        sitk_equa = sitk.AdaptiveHistogramEqualizationImageFilter()
        sitk_equa.SetAlpha(0.9)
        sitk_equa.SetBeta(0.9)
        sitk_equa.SetRadius(3)
        equa_img = sitk_equa.Execute(img)
        return equa_img
    
    def laplaciansharp_img(self,img):
        sitk_laplacian = sitk.LaplacianSharpeningImageFilter()
        sitk_laplacian.UseImageSpacingOn()
        img_lap = sitk_laplacian.Execute(img)
        return img_lap
    
    def threshlod_seg(self,img,upper,lower):
        thres_filter = sitk.BinaryThresholdImageFilter()
        thres_filter.SetLowerThreshold(lower)
        thres_filter.SetUpperThreshold(upper)
        thres_filter.SetOutsideValue(0)
        thres_filter.SetInsideValue(1)
        thres_mask = thres_filter.Execute(img)
        return thres_mask
    
    def guassian_blur(self,img):
        gussian_filter = sitk.SmoothingRecursiveGaussianImageFilter()
        gussian_filter.SetSigma(1.0)
        blur_img = gussian_filter.Execute(img)
        return blur_img
    
    def adaptive_region_grow(self,img,seed):
        confidence_filter = sitk.ConfidenceConnectedImageFilter()
        confidence_filter.SetSeedList(seed)
        confidence_filter.SetMultiplier(0.1)
        confidence_filter.SetNumberOfIterations(2)
        confidence_filter.SetInitialNeighborhoodRadius(2)
        mask = confidence_filter.Execute(img)
        return mask
    
    def region_grow(self,img,seed,upper,lower):
        connect_filter = sitk.ConnectedThresholdImageFilter()
        connect_filter.SetSeedList(seed)
        connect_filter.SetLower(lower)
        connect_filter.SetUpper(upper)
        mask = connect_filter.Execute(img)
        return mask
    
    def curvature_flow_image_filter(self,img):
        blur_filter = sitk.CurvatureFlowImageFilter()
        blur_filter.SetTimeStep(0.125)
        blur_filter.SetNumberOfIterations(2)
        blur_img  = blur_filter.Execute(img)
        return blur_img
    
    def mask_contour(self,mask):
        contour = sitk.LabelContour(mask)
        contour.SetSpacing(mask.GetSpacing())
        contour.SetDirection(mask.GetDirection())
        contour.SetOrigin(mask.GetOrigin())
        return contour
    
    def otsu_seg(self,img):
        otsu_filter = sitk.OtsuThresholdImageFilter()
        mask = otsu_filter.Execute(img)
        return mask
    
    def save_mask(self,img,array,file_path):
        cv_img = sitk.GetImageFromArray(array)
        cv_img.SetOrigin(img.GetOrigin())
        cv_img.SetSpacing(img.GetSpacing())
        cv_img.SetDirection(img.GetDirection())
        sitk.WriteImage(cv_img,file_path) 
    
    def adaptive_region_grow(self,img,seed):
        confidence_filter = sitk.ConfidenceConnectedImageFilter()
        confidence_filter.SetSeedList(seed)
        confidence_filter.SetMultiplier(1)
        confidence_filter.SetNumberOfIterations(2)
        confidence_filter.SetInitialNeighborhoodRadius(2)
        mask = confidence_filter.Execute(img)
        return mask
    

    
   
    