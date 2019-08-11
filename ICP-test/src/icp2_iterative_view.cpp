#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/common/transforms.h>
// #include <pcl/registration/icp_nl.h>

#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>
#include <vtkCamera.h>


void
loadFile(const char* fileName,
	 pcl::PointCloud<pcl::PointXYZ> &cloud
)
{
  pcl::PolygonMesh mesh;
  
  if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 )
  {
    PCL_ERROR ( "loadFile faild." );
    return;
  }
  else
    pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );
  
  // remove points having values of nan
  std::vector<int> index;
  pcl::removeNaNFromPointCloud ( cloud, cloud, index );
}

int main (int argc, char** argv)
{
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () );
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
  
  {
    // load source
    loadFile ( argv[1], *cloud_source );
    // load target
    loadFile ( argv[2], *cloud_target );
  }
  
  
  // transformed source ---> target
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_trans ( new pcl::PointCloud<pcl::PointXYZ> () );
  cloud_source_trans = cloud_source;
  
  
  boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
  viewer->setBackgroundColor (0, 0, 0);
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color ( cloud_source, 0, 255, 0 );
  viewer->addPointCloud<pcl::PointXYZ> (cloud_source, source_color, "source");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color ( cloud_target, 255, 255, 255 );
  viewer->addPointCloud<pcl::PointXYZ> ( cloud_target, target_color, "target");
  viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target" );
  
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_trans_color ( cloud_source_trans, 255, 0, 255 );
  viewer->addPointCloud<pcl::PointXYZ> ( cloud_source_trans, source_trans_color, "source trans" );
  viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source trans" );
  
  
  // orthographic (parallel) projection; same with pressing key 'o'
  viewer->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection( 1 );
  
  viewer->resetCamera();
  
  
  
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp ( new pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> () );
//   pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>::Ptr icp ( new pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> );
  icp->setMaximumIterations ( 1 );
  icp->setInputSource ( cloud_source_trans ); // not cloud_source, but cloud_source_trans!
  icp->setInputTarget ( cloud_target );
  
  
  Eigen::Matrix4f Transform = Eigen::Matrix4f::Identity();
//icp.getFinalTransformation();

 
Eigen::Matrix4f c2l_guess;
c2l_guess << 0, 0, 1, 0,
      -1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, 0, 1;

  int n=0;
  while ( !viewer->wasStopped() )
  {
    // registration
    icp->align ( *cloud_source_trans );
    
    if ( icp->hasConverged() )
    {
      viewer->updatePointCloud ( cloud_source_trans, source_trans_color, "source trans" );
      Transform = Transform * icp->getFinalTransformation();
      std::cout << "iter_"<<n++<<":  "<<icp->getFitnessScore() << std::endl <<Transform<<std::endl;
    }
    else
      std::cout << "Not converged." << std::endl;
    
std::cout << "final c2l: "<<std::endl <<Transform * c2l_guess<<std::endl;
std::cout << "final l2c: "<<std::endl <<(Transform * c2l_guess).inverse()<<std::endl;
    viewer->spinOnce();
  }
 
 
  
  return( 0 );
}

