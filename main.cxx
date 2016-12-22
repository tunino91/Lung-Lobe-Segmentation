
#include <sstream>
#include "itkImage.h"
#include "itkConfigure.h"
#include "itkAddImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkMaskImageFilter.h"
#include "itksys/SystemTools.hxx"
#include "itkImageRegionIterator.h"
#include "itkCurvatureFlowImageFilter.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkConnectedThresholdImageFilter.h"
#include "itkConfidenceConnectedImageFilter.h"
#include "itkDescoteauxSheetnessImageFilter.h"
#include "itkSymmetricEigenAnalysisImageFilter.h"
#include "itkSignedMaurerDistanceMapImageFilter.h"
#include "itkHessianRecursiveGaussianImageFilter.h"
#include "itkHessian3DToVesselnessMeasureImageFilter.h"

const    unsigned   int    Dimension = 3;
typedef  float             InputPixelType;
typedef  float             OutputPixelType;

typedef itk::Image < InputPixelType, Dimension >  InputImageType;
typedef itk::Image < OutputPixelType, Dimension > OutputImageType;
typedef itk::ImageFileReader < InputImageType > ReaderType;
typedef itk::ImageFileWriter < OutputImageType > WriterType;


typedef itk::CastImageFilter < InputImageType, OutputImageType > CastingFilterType;
typedef itk::CurvatureFlowImageFilter < InputImageType, OutputImageType > CurvatureFlowImageFilterType;
typedef itk::ConfidenceConnectedImageFilter< InputImageType, OutputImageType > ConnectedFilterType;
typedef itk::MaskImageFilter< InputImageType, OutputImageType > MaskFilterType;
typedef itk::ConnectedThresholdImageFilter< InputImageType,OutputImageType > ConnectedFilterType1;
typedef itk::BinaryThresholdImageFilter < InputImageType, OutputImageType > BinaryThresholdImageFilterType;

typedef itk::HessianRecursiveGaussianImageFilter< InputImageType > HessianFilterType;
typedef itk::Hessian3DToVesselnessMeasureImageFilter< OutputPixelType > VesselnessMeasureFilterType;
typedef HessianFilterType::OutputImageType                          HessianImageType;
typedef HessianImageType::PixelType                                 HessianPixelType;

typedef itk::FixedArray < double, HessianPixelType::Dimension >     EigenValueArrayType;
typedef itk::Image < EigenValueArrayType, Dimension >               EigenValueImageType;

typedef itk::SymmetricEigenAnalysisImageFilter < HessianImageType, EigenValueImageType > EigenAnalysisFilterType;
typedef itk::DescoteauxSheetnessImageFilter < EigenValueImageType, OutputImageType >   FilterType;
typedef itk::RescaleIntensityImageFilter < InputImageType,InputImageType >   RescalerType;
typedef itk::SignedMaurerDistanceMapImageFilter < InputImageType, OutputImageType > SignedMaurerDistanceMapImageFilterType;
typedef itk::AddImageFilter < InputImageType, InputImageType, OutputImageType > AddImageFilterType;

int main( int argc, char *argv[] )
{
    //////////////////////// SEGMENTATION ////////////////////////
    
    //// ConfidenceConnectedImageFilter is utilized for Lung binary segmentation ////

    ReaderType::Pointer reader1 = ReaderType::New();
    reader1->SetFileName( "preprocessed.mhd" ); // preprocessed Image
    reader1->Update();
    
    ReaderType::Pointer reader2 = ReaderType::New();
    ReaderType::Pointer reader3 = ReaderType::New();
    ReaderType::Pointer reader4 = ReaderType::New();
    ReaderType::Pointer reader5 = ReaderType::New();
    
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName( "LungBinaryMask.mhd" ); // Binary Image of Lung Mask
    
    CastingFilterType::Pointer caster = CastingFilterType::New();
    CurvatureFlowImageFilterType::Pointer smoothing = CurvatureFlowImageFilterType::New();
    ConnectedFilterType::Pointer confidenceConnected = ConnectedFilterType::New();

    smoothing->SetInput( reader1->GetOutput() );
    smoothing->SetNumberOfIterations( 2 );
    smoothing->SetTimeStep( 0.05 );
    
    confidenceConnected->SetInput( smoothing->GetOutput() );
    
    caster->SetInput( confidenceConnected->GetOutput() );
    caster->Update();
    
    writer->SetInput( caster->GetOutput() );
    
    InputImageType::IndexType index1;
    index1[0] = 130;
    index1[1] = 240;
    index1[2] = 192;

    InputImageType::IndexType index2;
    index2[0] = 386;
    index2[1] = 233;
    index2[2] = 192;
    
    confidenceConnected->SetMultiplier( 6 );
    confidenceConnected->SetNumberOfIterations( 10 );
    confidenceConnected->SetInitialNeighborhoodRadius( 2 );
    confidenceConnected->SetReplaceValue( 255 );
    confidenceConnected->AddSeed( index1 );
    confidenceConnected->AddSeed( index2 );

    writer->Update();
    //////////////////////// MASKING LUNG ////////////////////////
    
    //// MaskImageFilter is utilized to get the greayscale image of the lung ////
   
    MaskFilterType::Pointer maskFilter = MaskFilterType::New();
    
    reader1->SetFileName( "preprocessed.mhd" ); // Preprocessed Image
    reader2->SetFileName( "LungBinaryMask.mhd" );  // Binary Mask
    writer->SetFileName( "MaskedLung.mhd" );  // Masked output
    
    reader1->Update();
    reader2->Update();
    
    maskFilter->SetInput(reader1->GetOutput());
    maskFilter->SetMaskImage(reader2->GetOutput());
    maskFilter->Update();
    
    writer->SetInput(maskFilter->GetOutput());
    writer->Update();
    
    //////////////////////// BRONCHI SEGMENTATION ////////////////////////
    
    //// ConnectedThresholdImageFilter is utilized to get binary segmentation of bronchi ////
    ConnectedFilterType1::Pointer connectedThreshold = ConnectedFilterType1::New();
    
    reader1->SetFileName( "MaskedLung.mhd" );
    reader1->Update();
    
    writer->SetFileName( "BronchiSegmentation.mhd" );
    
    smoothing->SetInput( reader1->GetOutput() );
    connectedThreshold->SetInput( smoothing->GetOutput() );
    caster->SetInput( connectedThreshold->GetOutput() );
    writer->SetInput( caster->GetOutput() );
    
    smoothing->SetNumberOfIterations( 5 );
    smoothing->SetTimeStep( 0.125 );
    
    const InputPixelType lowerThreshold = atof( "-1020" ); // The threshold values are put experimentally by looking at the lung image.
    const InputPixelType upperThreshold = atof( "-890" );
    connectedThreshold->SetLower(  lowerThreshold  );
    connectedThreshold->SetUpper(  upperThreshold  );
    
    connectedThreshold->SetReplaceValue( 255 );
    
    InputImageType::IndexType  index3; // The index value is put experimentally by looking at the lung image.
    index3[0] = atoi( "270" );
    index3[1] = atoi( "229" );
    index3[2] = atoi( "443" );
    connectedThreshold->SetSeed( index3 );
    
    writer->Update();
   
    //////////////////////// INVERT BINARY BRONCHI ////////////////////////
    
    //// BinaryThresholdImageFilter is utilized to invert the background to foreground and vice-versa ////
    
    int lowerThreshold1 = 254;
    int upperThreshold1 = 256;
    
    reader1->SetFileName("BronchiSegmentation.mhd");
    writer->SetFileName("InvertedBronchiSegmentation.mhd");
    
    reader1->Update();

    BinaryThresholdImageFilterType::Pointer thresholdFilter = BinaryThresholdImageFilterType::New();
    
    thresholdFilter->SetInput(reader1->GetOutput());
    thresholdFilter->SetLowerThreshold(lowerThreshold1);
    thresholdFilter->SetUpperThreshold(upperThreshold1);
    thresholdFilter->SetInsideValue(0);
    thresholdFilter->SetOutsideValue(255);
    writer->SetInput(thresholdFilter->GetOutput());
    
    writer->Update();
    
   //////////////////////// TAKE OUT BRONCHI FROM LUNG ////////////////////////
   
    reader1->SetFileName( "MaskedLung.mhd" ); // Preprocessed Image
    reader2->SetFileName( "InvertedBronchiSegmentation.mhd" );  // Binary Mask
    writer->SetFileName( "BronchilessLung.mhd" );  // Masked output
    
    reader1->Update();
    reader2->Update();
    
    maskFilter->SetInput(reader1->GetOutput());
    maskFilter->SetMaskImage(reader2->GetOutput());
    maskFilter->Update();
    
    writer->SetInput(maskFilter->GetOutput());
    writer->Update();
   
    
    //////////////////////// VESSELS ////////////////////////
   //// Hessian3DToVesselnessMeasureImageFilter is utilized to get a greyscale vessels ////
    
    HessianFilterType::Pointer hessianFilter = HessianFilterType::New();
    VesselnessMeasureFilterType::Pointer vesselnessFilter = VesselnessMeasureFilterType::New();
    
    reader1->SetFileName( "BronchilessLung.mhd" );
    reader1->Update();
  
    hessianFilter->SetInput( reader1->GetOutput() );
    hessianFilter->SetSigma( atof( "1.5" ) );
    
    vesselnessFilter->SetInput( hessianFilter->GetOutput() );
    vesselnessFilter->SetAlpha1( atof( "0.5" ) );
    vesselnessFilter->SetAlpha2( atof( "0.5" ) );
   
    writer->SetInput( vesselnessFilter->GetOutput() );
    writer->SetFileName( "Vessels.mhd" );
    writer->Update();
    
    //////////////////////// CONVERT BINARY ////////////////////////
    //// BinaryThresholdImageFilter is utilized to get a binary vessels ////
    
    int lowerThreshold2 = 50; // The threshold values are determined experimentaly.
    int upperThreshold2 = 256;
    
    reader1->SetFileName("Vessels.mhd");  // Input greyscale vessels
    writer->SetFileName("VesselBinary.mhd");  // Output binary vessels
    
    reader1->Update();
   

    thresholdFilter->SetInput(reader1->GetOutput());
    thresholdFilter->SetLowerThreshold(lowerThreshold2);
    thresholdFilter->SetUpperThreshold(upperThreshold2);
    thresholdFilter->SetInsideValue(255);
    thresholdFilter->SetOutsideValue(0);
    writer->SetInput(thresholdFilter->GetOutput());
    
    writer->Update();
    
    //////////////////////// SHEET ////////////////////////
    //// DescoteauxSheetnessImageFilter is utilized to get sheet-like structures ////
        
    HessianFilterType::Pointer hessian = HessianFilterType::New();
    EigenAnalysisFilterType::Pointer eigen = EigenAnalysisFilterType::New();
    FilterType::Pointer sheetnessFilter = FilterType::New();

    reader1->SetFileName( "MaskedLung.mhd" );
    reader1->Update();
 
    hessian->SetInput( reader1->GetOutput() );
    hessian->SetSigma( atof( "0.6" ) );// Smoothness
    
    eigen->SetInput( hessian->GetOutput() );
    eigen->SetDimension( Dimension ); // Get the eigen values
    
    sheetnessFilter->SetInput( eigen->GetOutput() );
    sheetnessFilter->SetDetectBrightSheets(true);
    sheetnessFilter->InPlaceOn();
    sheetnessFilter->SetDetectBrightSheets( atoi( "1" ) ); // Indicates that I want to find bright structures
    sheetnessFilter->SetSheetnessNormalization( atof( "0.3" ) );
    sheetnessFilter->SetBloobinessNormalization( atof( "1" ) );
    sheetnessFilter->SetNoiseNormalization( atof( "15" ) );

    writer->SetFileName( "Sheetness.mhd" );
    writer->SetInput( sheetnessFilter->GetOutput() );
    writer->Update();
   
    //////////////////////// RESCALE ////////////////////////
    //// RescaleIntensityImageFilter is utilized to map the image to 0-255 ////
   
    reader1->SetFileName( "Sheetness.mhd" );
    reader1->Update();

   
    RescalerType::Pointer rescaler1 = RescalerType::New();
    rescaler1->SetOutputMinimum(  0  );
    rescaler1->SetOutputMaximum( 255 );
    rescaler1->SetInput( reader1->GetOutput() );
    rescaler1->Update();
    
    writer->SetFileName( "SheetnessRescaled.mhd" );
    writer->SetInput( rescaler1->GetOutput() );
    writer->Update();
    
    //////////////////////// FIND FISSURE ////////////////////////
    //// ConnectedThresholdImageFilter is utilized to get binary image ////
  
    
    
    reader1->SetFileName( "SheetnessRescaled.mhd" );
    reader1->Update();
    
    writer->SetFileName( "FissureBinary.mhd" );

    smoothing->SetInput( reader1->GetOutput() );
    connectedThreshold->SetInput( smoothing->GetOutput() );
    caster->SetInput( connectedThreshold->GetOutput() );
    writer->SetInput( caster->GetOutput() );
    
    smoothing->SetNumberOfIterations( 5 );
    smoothing->SetTimeStep( 0.125 );
    
    const InputPixelType lowerThreshold4 = atof( "110" ); // The pixel values are determined experimentaly.
    const InputPixelType upperThreshold4 = atof( "255" );
    
    connectedThreshold->SetLower(  lowerThreshold4  );
    connectedThreshold->SetUpper(  upperThreshold4  );
    connectedThreshold->SetReplaceValue( 255 );
    
    InputImageType::IndexType  index4;
    index4[0] = atoi( "152" );
    index4[1] = atoi( "226" );
    index4[2] = atoi( "240" );
    connectedThreshold->SetSeed( index4 );
    
    writer->Update();
    //////////////////////// INVERT FISSURE ////////////////////////
    //// BinaryThresholdImageFilter is utilized to invert the background to foreground and vice-versa ////
   
    int lowerThreshold3 = 254;
    int upperThreshold3 = 256;
    
    reader1->SetFileName("FissureBinary.mhd");
    reader1->Update();
    
    writer->SetFileName("InvertedFissureBinary.mhd");
    
    thresholdFilter->SetInput(reader1->GetOutput());
    thresholdFilter->SetLowerThreshold(lowerThreshold3);
    thresholdFilter->SetUpperThreshold(upperThreshold3);
    thresholdFilter->SetInsideValue(0);
    thresholdFilter->SetOutsideValue(255);
    
    writer->SetInput(thresholdFilter->GetOutput());
    writer->Update();
    
    //////////////////////// DISTANCE TRANSFORM IMAGE ////////////////////////
    //// SignedMaurerDistanceMapImageFilter is utilized to find distance images of bronchi, vessels and fissures ////
   
    SignedMaurerDistanceMapImageFilterType::Pointer distanceMapImageFilter = SignedMaurerDistanceMapImageFilterType::New();
    
    reader1->SetFileName("BronchiSegmentation.mhd");
    reader1->Update();
    
    distanceMapImageFilter->SetInput(reader1->GetOutput());
    
    writer->SetFileName("DistanceBronchiScattered.mhd");
    writer->SetInput(distanceMapImageFilter->GetOutput());
    writer->Update();
    
    reader1->SetFileName("VesselBinary.mhd");
    reader1->Update();
    
    distanceMapImageFilter->SetInput(reader1->GetOutput());
    
    writer->SetFileName("DistanceVesselScattered.mhd");
    writer->SetInput(distanceMapImageFilter->GetOutput());
    writer->Update();
    
    reader1->SetFileName("InvertedFissureBinary.mhd");
    reader1->Update();
    
    distanceMapImageFilter->SetInput(reader1->GetOutput());
    
    writer->SetFileName("DistanceFissureScattered.mhd");
    writer->SetInput(distanceMapImageFilter->GetOutput());
    writer->Update();
    
    
    //////////////////////// RESCALE ////////////////////////
    //// RescaleIntensityImageFilter is utilized to map the image to 0-255 ////
 
    reader1->SetFileName( "DistanceBronchiScattered.mhd" );
    reader1->Update();
    
    rescaler1->SetOutputMinimum(  0  );
    rescaler1->SetOutputMaximum( 255 );
    rescaler1->SetInput( reader1->GetOutput() );
    rescaler1->Update();
    
    writer->SetFileName( "DistanceBronchiScatteredRescaled.mhd" );
    writer->SetInput( rescaler1->GetOutput() );
    writer->Update();
    
    reader1->SetFileName( "DistanceVesselScattered.mhd" );
    reader1->Update();
    
    rescaler1->SetOutputMinimum(  0  );
    rescaler1->SetOutputMaximum( 255 );
    rescaler1->SetInput( reader1->GetOutput() );
    rescaler1->Update();
    
    writer->SetFileName( "DistanceVesselScatteredRescaled.mhd" );
    writer->SetInput( rescaler1->GetOutput() );
    writer->Update();
    
    //////////////////////// MASK DISTANCE IMAGES ////////////////////////
  
    reader1->SetFileName( "DistanceBronchiScatteredRescaled.mhd" ); // Image
    reader1->Update();
    
    reader2->SetFileName( "LungBinaryMask.mhd" );  // Binary Mask
    reader2->Update();
    
    writer->SetFileName( "DistanceBronchiMasked.mhd" );  // Masked output

    maskFilter->SetInput(reader1->GetOutput());
    maskFilter->SetMaskImage(reader2->GetOutput());
    maskFilter->Update();
    
    writer->SetInput(maskFilter->GetOutput());
    writer->Update();
    
    reader1->SetFileName( "DistanceVesselScatteredRescaled.mhd" ); // Image
    reader1->Update();
    
    reader2->SetFileName( "LungBinaryMask.mhd" );  // Binary Mask
    reader2->Update();
    
    writer->SetFileName( "DistanceVesselMasked.mhd" );  // Masked output
    
    maskFilter->SetInput(reader1->GetOutput());
    maskFilter->SetMaskImage(reader2->GetOutput());
    maskFilter->Update();
    
    writer->SetInput(maskFilter->GetOutput());
    writer->Update();
    
    reader1->SetFileName( "DistanceFissureScattered.mhd" ); // Image
    reader1->Update();
    
    reader2->SetFileName( "LungBinaryMask.mhd" );  // Binary Mask
    reader2->Update();
    
    writer->SetFileName( "DistanceFissureMasked.mhd" );  // Masked output
    
    maskFilter->SetInput(reader1->GetOutput());
    maskFilter->SetMaskImage(reader2->GetOutput());
    maskFilter->Update();
    
    writer->SetInput(maskFilter->GetOutput());
    writer->Update();
    
    reader1->SetFileName( "DistanceFissureMasked.mhd" );
    reader1->Update();
    rescaler1->SetOutputMinimum(  0  );
    rescaler1->SetOutputMaximum( 255 );
    rescaler1->SetInput( reader1->GetOutput() );
    rescaler1->Update();
    writer->SetFileName( "DistanceFissureMaskedRescaled.mhd" );
    writer->SetInput( rescaler1->GetOutput() );
    writer->Update();
   
    //////////////////////// CALCULATE THE COST IMAGE ////////////////////////

    //// Rescale the original masked lung and add it to rescaled original masked image, the distance images of bronchi-vessels-fissure and fissure segmentation

    reader1->SetFileName( "MaskedLung.mhd" );
    reader1->Update();
    
    rescaler1->SetOutputMinimum(  0  );
    rescaler1->SetOutputMaximum( 255 );
    rescaler1->SetInput( reader1->GetOutput() );
    rescaler1->Update();
    
    writer->SetFileName( "MaskedRescaled.mhd" );
    writer->SetInput( rescaler1->GetOutput() );
    writer->Update();
    
    reader1->SetFileName( "MaskedRescaled.mhd" );  // masked
    reader2->SetFileName( "DistanceVesselMasked.mhd" );  // DistanceVessel
    reader3->SetFileName( "DistanceBronchiMasked.mhd" );  // DistanceBronchi
    reader4->SetFileName( "DistanceFissureMaskedRescaled.mhd" );  // DistanceFissure
    reader5->SetFileName( "FissureBinary.mhd" );  // fissure binary
    
    writer->SetFileName( "CostImage.mhd" );
    
    AddImageFilterType::Pointer addFilter = AddImageFilterType::New ();
    
    addFilter->SetInput1(reader1->GetOutput());
    addFilter->SetInput2(reader2->GetOutput());
    addFilter->Update();
    
    addFilter->SetInput1(addFilter->GetOutput());
    addFilter->SetInput2(reader3->GetOutput());
    addFilter->Update();
    
    addFilter->SetInput1(addFilter->GetOutput());
    addFilter->SetInput2(reader4->GetOutput());
    addFilter->Update();
    
    addFilter->SetInput1(addFilter->GetOutput());
    addFilter->SetInput2(reader5->GetOutput());
    addFilter->Update();
    rescaler1->SetOutputMinimum(  0  );
    rescaler1->SetOutputMaximum( 255 );
    rescaler1->SetInput( addFilter->GetOutput() );
    rescaler1->Update();
    
    writer->SetInput(rescaler1->GetOutput());
    writer->Update();


  return EXIT_SUCCESS;
}
