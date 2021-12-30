Video: https://youtu.be/IEgP-Lgx6bA

## 三维点云特征线提取DEMO程序源代码使用说明
****************************************************************************
* 工程名称：FeatureLineExtractor
* 程序名称：Feature Line Extractor
* 运行环境：Windows x64
* 编译环境：Visual Studio 2017+PCL1.8.1+Qt5.10.1(msvc2017_64)
****************************************************************************

1. Qt+Visual Studio环境配置说明

(1) 安装Qt

        Qt编译器版本必须为系统相应位数的msvc编译器，msvc版本必须与Visual Studio编译器版本一致。

(2) 在Visual Studio中安装Qt插件

        在Visual Studio中打开工具-扩展和更新，在联机-Visual Studio Marketplace中搜索并安装插件：Qt Visual Studio Tools

2. PCL环境配置说明

(1) 下载PCL安装程序和PDB包

        GitHub地址：https://github.com/PointCloudLibrary/pcl/releases
        
        分别下载相应系统位数和版本的PCL安装程序和PDB包，两者系统位数（32/64）、PCL版本号和msvc版本号必须相同，msvc版本号必须与Visual Studio编译器版本一致。

(2) 安装PCL

        运行PCL-x.x.x-AllInOne-msvcxxxx-winxx.exe，选择合适的环境变量配置，选择安装路径（安装路径在下文简称为%pcl_path%），之后第三方库的安装路径不用改变，都安装于%pcl_path%\3rdParty\目录之下即可。

(3) 安装PDB包

        解压pcl-x.x.x-pdb-msvcxxxx-winxx.zip，将里面所有的pdb文件拷贝到%pcl_path%\bin目录下。

(4) 配置第三方PCL库环境变量

        在系统变量Pah中添加环境变量：%PCL_ROOT%\bin;%PCL_ROOT%\3rdParty\FLANN\bin;%PCL_ROOT%\3rdParty\VTK\bin;%PCL_ROOT%\Qhull\bin;%PCL_ROOT%\3rdParty\OpenNI2\Tools
        重启系统使配置生效。

3. Visual Studio工程的配置

(1) Visual Studio解决方案类型为Visual C++-Qt-Qt GUI Application。

(2) 在项目属性-VC++目录-可执行文件目录中添加以下位置：
·Debug和Release配置相同。
%pcl_path%\bin
%pcl_path%\3rdParty\Boost\lib
%pcl_path%\3rdParty\FLANN\bin
%pcl_path%\3rdParty\Qhull\bin
%pcl_path%\3rdParty\VTK\bin

(3) 在项目属性-VC++目录-包含目录中添加以下位置：
·Debug和Release配置相同。
%pcl_path%\include\pcl-1.8
%pcl_path%\3rdParty\Boost\include\boost-1_64
%pcl_path%\3rdParty\Qhull\include
%pcl_path%\3rdParty\Eigen\eigen3
%pcl_path%\3rdParty\VTK\include\vtk-8.0
%pcl_path%\3rdParty\FLANN\include

(4) 在项目属性-VC++目录-库目录中添加以下位置：
·Debug和Release配置相同。
%pcl_path%\lib
%pcl_path%\3rdParty\Boost\lib
%pcl_path%\3rdParty\VTK\lib
%pcl_path%\3rdParty\FLANN\lib
%pcl_path%\3rdParty\Qhull\lib

(5) 在项目属性-链接器-输入-附加依赖项中添加以下位置：
·Debug配置：
pcl_common_debug.lib
pcl_features_debug.lib
pcl_filters_debug.lib
pcl_io_debug.lib
pcl_io_ply_debug.lib
pcl_kdtree_debug.lib
pcl_keypoints_debug.lib
pcl_ml_debug.lib
pcl_octree_debug.lib
pcl_outofcore_debug.lib
pcl_people_debug.lib
pcl_recognition_debug.lib
pcl_registration_debug.lib
pcl_sample_consensus_debug.lib
pcl_search_debug.lib
pcl_segmentation_debug.lib
pcl_stereo_debug.lib
pcl_surface_debug.lib
pcl_tracking_debug.lib
pcl_visualization_debug.lib
libboost_atomic-vc141-mt-gd-1_64.lib
libboost_bzip2-vc141-mt-gd-1_64.lib
libboost_chrono-vc141-mt-gd-1_64.lib
libboost_container-vc141-mt-gd-1_64.lib
libboost_context-vc141-mt-gd-1_64.lib
libboost_coroutine-vc141-mt-gd-1_64.lib
libboost_date_time-vc141-mt-gd-1_64.lib
libboost_exception-vc141-mt-gd-1_64.lib
libboost_fiber-vc141-mt-gd-1_64.lib
libboost_filesystem-vc141-mt-gd-1_64.lib
libboost_graph_parallel-vc141-mt-gd-1_64.lib
libboost_graph-vc141-mt-gd-1_64.lib
libboost_iostreams-vc141-mt-gd-1_64.lib
libboost_locale-vc141-mt-gd-1_64.lib
libboost_log-vc141-mt-gd-1_64.lib
libboost_log_setup-vc141-mt-gd-1_64.lib
libboost_math_c99-vc141-mt-gd-1_64.lib
libboost_math_c99f-vc141-mt-gd-1_64.lib
libboost_math_c99l-vc141-mt-gd-1_64.lib
libboost_math_tr1-vc141-mt-gd-1_64.lib
libboost_math_tr1f-vc141-mt-gd-1_64.lib
libboost_math_tr1l-vc141-mt-gd-1_64.lib
libboost_mpi-vc141-mt-gd-1_64.lib
libboost_numpy3-vc141-mt-gd-1_64.lib
libboost_numpy-vc141-mt-gd-1_64.lib
libboost_prg_exec_monitor-vc141-mt-gd-1_64.lib
libboost_program_options-vc141-mt-gd-1_64.lib
libboost_python3-vc141-mt-gd-1_64.lib
libboost_python-vc141-mt-gd-1_64.lib
libboost_random-vc141-mt-gd-1_64.lib
libboost_regex-vc141-mt-gd-1_64.lib
libboost_serialization-vc141-mt-gd-1_64.lib
libboost_signals-vc141-mt-gd-1_64.lib
libboost_system-vc141-mt-gd-1_64.lib
libboost_test_exec_monitor-vc141-mt-gd-1_64.lib
libboost_thread-vc141-mt-gd-1_64.lib
libboost_timer-vc141-mt-gd-1_64.lib
libboost_type_erasure-vc141-mt-gd-1_64.lib
libboost_unit_test_framework-vc141-mt-gd-1_64.lib
libboost_wave-vc141-mt-gd-1_64.lib
libboost_wserialization-vc141-mt-gd-1_64.lib
libboost_zlib-vc141-mt-gd-1_64.lib
vtkzlib-8.0-gd.lib
vtkViewsContext2D-8.0-gd.lib
vtkhdf5_hl-8.0-gd.lib
vtkFiltersProgrammable-8.0-gd.lib
vtkIOMovie-8.0-gd.lib
vtkexpat-8.0-gd.lib
vtkRenderingLOD-8.0-gd.lib
vtkIOPLY-8.0-gd.lib
vtkpng-8.0-gd.lib
vtkIOVideo-8.0-gd.lib
vtkRenderingImage-8.0-gd.lib
vtkFiltersVerdict-8.0-gd.lib
vtkjpeg-8.0-gd.lib
vtkInteractionImage-8.0-gd.lib
vtkverdict-8.0-gd.lib
vtkoggtheora-8.0-gd.lib
vtkImagingStatistics-8.0-gd.lib
vtkCommonSystem-8.0-gd.lib
vtkFiltersParallelImaging-8.0-gd.lib
vtkFiltersTexture-8.0-gd.lib
vtkCommonMisc-8.0-gd.lib
vtkCommonMath-8.0-gd.lib
vtkFiltersSelection-8.0-gd.lib
vtkImagingFourier-8.0-gd.lib
vtkIOXMLParser-8.0-gd.lib
vtkproj4-8.0-gd.lib
vtkalglib-8.0-gd.lib
vtktiff-8.0-gd.lib
vtksqlite-8.0-gd.lib
vtkImagingSources-8.0-gd.lib
vtkCommonTransforms-8.0-gd.lib
vtkImagingMath-8.0-gd.lib
vtkFiltersGeneric-8.0-gd.lib
vtkexoIIc-8.0-gd.lib
vtkfreetype-8.0-gd.lib
vtkImagingColor-8.0-gd.lib
vtknetcdf_c++-gd.lib
vtkNetCDF-8.0-gd.lib
vtkFiltersImaging-8.0-gd.lib
vtkFiltersHyperTree-8.0-gd.lib
vtkImagingMorphological-8.0-gd.lib
vtkImagingStencil-8.0-gd.lib
vtkRenderingFreeType-8.0-gd.lib
vtkCommonColor-8.0-gd.lib
vtkjsoncpp-8.0-gd.lib
vtkRenderingContext2D-8.0-gd.lib
vtkFiltersAMR-8.0-gd.lib
vtkRenderingContextOpenGL-8.0-gd.lib
vtkIOParallelXML-8.0-gd.lib
vtkViewsCore-8.0-gd.lib
vtkImagingHybrid-8.0-gd.lib
vtkIOExport-8.0-gd.lib
vtkRenderingVolumeOpenGL-8.0-gd.lib
vtkIOImport-8.0-gd.lib
vtkDomainsChemistry-8.0-gd.lib
vtkInteractionStyle-8.0-gd.lib
vtkCommonComputationalGeometry-8.0-gd.lib
vtkDICOMParser-8.0-gd.lib
vtklibxml2-8.0-gd.lib
vtkIOMINC-8.0-gd.lib
vtkIOSQL-8.0-gd.lib
vtkFiltersModeling-8.0-gd.lib
vtkParallelCore-8.0-gd.lib
vtkIOAMR-8.0-gd.lib
vtkGeovisCore-8.0-gd.lib
vtksys-8.0-gd.lib
vtkIOEnSight-8.0-gd.lib
vtkImagingGeneral-8.0-gd.lib
vtkFiltersFlowPaths-8.0-gd.lib
vtkIOLSDyna-8.0-gd.lib
vtkFiltersSMP-8.0-gd.lib
vtkIOParallel-8.0-gd.lib
vtkFiltersGeometry-8.0-gd.lib
vtkInfovisLayout-8.0-gd.lib
vtkIOCore-8.0-gd.lib
vtkFiltersSources-8.0-gd.lib
vtkIOLegacy-8.0-gd.lib
vtkFiltersHybrid-8.0-gd.lib
vtkIONetCDF-8.0-gd.lib
vtkFiltersExtraction-8.0-gd.lib
vtkFiltersParallel-8.0-gd.lib
vtkRenderingLabel-8.0-gd.lib
vtkIOInfovis-8.0-gd.lib
vtkRenderingAnnotation-8.0-gd.lib
vtkImagingCore-8.0-gd.lib
vtkCommonExecutionModel-8.0-gd.lib
vtkhdf5-8.0-gd.lib
vtkViewsInfovis-8.0-gd.lib
vtkIOGeometry-8.0-gd.lib
vtkmetaio-8.0-gd.lib
vtkIOImage-8.0-gd.lib
vtkIOXML-8.0-gd.lib
vtkIOExodus-8.0-gd.lib
vtkRenderingVolume-8.0-gd.lib
vtkChartsCore-8.0-gd.lib
vtkFiltersStatistics-8.0-gd.lib
vtkInfovisCore-8.0-gd.lib
vtkRenderingCore-8.0-gd.lib
vtkInteractionWidgets-8.0-gd.lib
vtkRenderingOpenGL-8.0-gd.lib
vtkFiltersGeneral-8.0-gd.lib
vtkFiltersCore-8.0-gd.lib
vtkCommonDataModel-8.0-gd.lib
vtkCommonCore-8.0-gd.lib
flann_cpp_s-gd.lib
qhullstatic_d.lib

·Release配置：
pcl_common_release.lib
pcl_features_release.lib
pcl_filters_release.lib
pcl_io_release.lib
pcl_io_ply_release.lib
pcl_kdtree_release.lib
pcl_keypoints_release.lib
pcl_ml_release.lib
pcl_octree_release.lib
pcl_outofcore_release.lib
pcl_people_release.lib
pcl_recognition_release.lib
pcl_registration_release.lib
pcl_sample_consensus_release.lib
pcl_search_release.lib
pcl_segmentation_release.lib
pcl_stereo_release.lib
pcl_surface_release.lib
pcl_tracking_release.lib
pcl_visualization_release.lib
libboost_atomic-vc141-mt-1_64.lib
libboost_bzip2-vc141-mt-1_64.lib
libboost_chrono-vc141-mt-1_64.lib
libboost_container-vc141-mt-1_64.lib
libboost_context-vc141-mt-1_64.lib
libboost_coroutine-vc141-mt-1_64.lib
libboost_date_time-vc141-mt-1_64.lib
libboost_exception-vc141-mt-1_64.lib
libboost_fiber-vc141-mt-1_64.lib
libboost_filesystem-vc141-mt-1_64.lib
libboost_graph_parallel-vc141-mt-1_64.lib
libboost_graph-vc141-mt-1_64.lib
libboost_iostreams-vc141-mt-1_64.lib
libboost_locale-vc141-mt-1_64.lib
libboost_log-vc141-mt-1_64.lib
libboost_log_setup-vc141-mt-1_64.lib
libboost_math_c99-vc141-mt-1_64.lib
libboost_math_c99f-vc141-mt-1_64.lib
libboost_math_c99l-vc141-mt-1_64.lib
libboost_math_tr1-vc141-mt-1_64.lib
libboost_math_tr1f-vc141-mt-1_64.lib
libboost_math_tr1l-vc141-mt-1_64.lib
libboost_mpi-vc141-mt-1_64.lib
libboost_numpy3-vc141-mt-1_64.lib
libboost_numpy-vc141-mt-1_64.lib
libboost_prg_exec_monitor-vc141-mt-1_64.lib
libboost_program_options-vc141-mt-1_64.lib
libboost_python3-vc141-mt-1_64.lib
libboost_python-vc141-mt-1_64.lib
libboost_random-vc141-mt-1_64.lib
libboost_regex-vc141-mt-1_64.lib
libboost_serialization-vc141-mt-1_64.lib
libboost_signals-vc141-mt-1_64.lib
libboost_system-vc141-mt-1_64.lib
libboost_test_exec_monitor-vc141-mt-1_64.lib
libboost_thread-vc141-mt-1_64.lib
libboost_timer-vc141-mt-1_64.lib
libboost_type_erasure-vc141-mt-1_64.lib
libboost_unit_test_framework-vc141-mt-1_64.lib
libboost_wave-vc141-mt-1_64.lib
libboost_wserialization-vc141-mt-1_64.lib
libboost_zlib-vc141-mt-1_64.lib
vtkzlib-8.0.lib
vtkViewsContext2D-8.0.lib
vtkhdf5_hl-8.0.lib
vtkFiltersProgrammable-8.0.lib
vtkIOMovie-8.0.lib
vtkexpat-8.0.lib
vtkRenderingLOD-8.0.lib
vtkIOPLY-8.0.lib
vtkpng-8.0.lib
vtkIOVideo-8.0.lib
vtkRenderingImage-8.0.lib
vtkFiltersVerdict-8.0.lib
vtkjpeg-8.0.lib
vtkInteractionImage-8.0.lib
vtkverdict-8.0.lib
vtkoggtheora-8.0.lib
vtkImagingStatistics-8.0.lib
vtkCommonSystem-8.0.lib
vtkFiltersParallelImaging-8.0.lib
vtkFiltersTexture-8.0.lib
vtkCommonMisc-8.0.lib
vtkCommonMath-8.0.lib
vtkFiltersSelection-8.0.lib
vtkImagingFourier-8.0.lib
vtkIOXMLParser-8.0.lib
vtkproj4-8.0.lib
vtkalglib-8.0.lib
vtktiff-8.0.lib
vtksqlite-8.0.lib
vtkImagingSources-8.0.lib
vtkCommonTransforms-8.0.lib
vtkImagingMath-8.0.lib
vtkFiltersGeneric-8.0.lib
vtkexoIIc-8.0.lib
vtkfreetype-8.0.lib
vtkImagingColor-8.0.lib
vtknetcdf_c++.lib
vtkNetCDF-8.0.lib
vtkFiltersImaging-8.0.lib
vtkFiltersHyperTree-8.0.lib
vtkImagingMorphological-8.0.lib
vtkImagingStencil-8.0.lib
vtkRenderingFreeType-8.0.lib
vtkCommonColor-8.0.lib
vtkjsoncpp-8.0.lib
vtkRenderingContext2D-8.0.lib
vtkFiltersAMR-8.0.lib
vtkRenderingContextOpenGL-8.0.lib
vtkIOParallelXML-8.0.lib
vtkViewsCore-8.0.lib
vtkImagingHybrid-8.0.lib
vtkIOExport-8.0.lib
vtkRenderingVolumeOpenGL-8.0.lib
vtkIOImport-8.0.lib
vtkDomainsChemistry-8.0.lib
vtkInteractionStyle-8.0.lib
vtkCommonComputationalGeometry-8.0.lib
vtkDICOMParser-8.0.lib
vtklibxml2-8.0.lib
vtkIOMINC-8.0.lib
vtkIOSQL-8.0.lib
vtkFiltersModeling-8.0.lib
vtkParallelCore-8.0.lib
vtkIOAMR-8.0.lib
vtkGeovisCore-8.0.lib
vtksys-8.0.lib
vtkIOEnSight-8.0.lib
vtkImagingGeneral-8.0.lib
vtkFiltersFlowPaths-8.0.lib
vtkIOLSDyna-8.0.lib
vtkFiltersSMP-8.0.lib
vtkIOParallel-8.0.lib
vtkFiltersGeometry-8.0.lib
vtkInfovisLayout-8.0.lib
vtkIOCore-8.0.lib
vtkFiltersSources-8.0.lib
vtkIOLegacy-8.0.lib
vtkFiltersHybrid-8.0.lib
vtkIONetCDF-8.0.lib
vtkFiltersExtraction-8.0.lib
vtkFiltersParallel-8.0.lib
vtkRenderingLabel-8.0.lib
vtkIOInfovis-8.0.lib
vtkRenderingAnnotation-8.0.lib
vtkImagingCore-8.0.lib
vtkCommonExecutionModel-8.0.lib
vtkhdf5-8.0.lib
vtkViewsInfovis-8.0.lib
vtkIOGeometry-8.0.lib
vtkmetaio-8.0.lib
vtkIOImage-8.0.lib
vtkIOXML-8.0.lib
vtkIOExodus-8.0.lib
vtkRenderingVolume-8.0.lib
vtkChartsCore-8.0.lib
vtkFiltersStatistics-8.0.lib
vtkInfovisCore-8.0.lib
vtkRenderingCore-8.0.lib
vtkInteractionWidgets-8.0.lib
vtkRenderingOpenGL-8.0.lib
vtkFiltersGeneral-8.0.lib
vtkFiltersCore-8.0.lib
vtkCommonDataModel-8.0.lib
vtkCommonCore-8.0.lib
flann_cpp_s.lib
qhullstatic.lib

(6) 其他问题及注意事项请参阅：https://www.cnblogs.com/li-yao7758258/p/8066352.html
