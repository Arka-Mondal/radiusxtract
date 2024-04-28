#include <cstdlib>
#include <cmath>

#include <iostream>

#include <QApplication>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <vtk/QVTKOpenGLNativeWidget.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>

#include <vtkActor.h>
#include <vtkNamedColors.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>

static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
static bool isLoaded = false;

void calculateRadius(void) {
  extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  std::cout << "Calculating...\n";

  // Estimate surface normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.03); // adjust this parameter as needed
  ne.compute (*normals);

  // Concatenate the XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

  // Create the segmentation object for cylinder segmentation and set parameters
  pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::Normal> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1); // set acceptable range for cylinder radius
  seg.setInputCloud (cloud_with_normals);
  seg.setInputNormals (normals);

  // Obtain cylinder inliers and coefficients
  seg.segment (*inliers, *coefficients);

  // Fit cylinder using inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_points (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);
  extract.setNegative (false);
  extract.filter (*cylinder_points);

  // Estimate radius of curvature
  double radius = std::abs(coefficients->values[6]);

  std::cout << "Radius of the cylinder: " << radius << '\n';

  std::cout << "Done\n";
}

void loadObject(QString &fileName, vtkRenderer* renderer, vtkGenericOpenGLRenderWindow* renderWindow) {

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *cloud) == -1) {
    // PCL_ERROR("Couldn't read PCD file!\n");
    std::exit(EXIT_FAILURE);
  }

  isLoaded = true;

  pcl::io::pointCloudTovtkPolyData(*cloud, polydata.Get());

  mapper->SetInputData(polydata);
  actor->SetMapper(mapper);

  renderer->RemoveAllViewProps();

  renderer->AddActor(actor);
  renderer->ResetCamera();

  renderWindow->Render();
}



int main(int argc, char* argv[]) {
    std::string home_dir(std::getenv("HOME"));


    vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();

    // Set up the Qt application
    QApplication app(argc, argv);

    // Create the main window
    QWidget mainWindow;
    mainWindow.setWindowTitle("VTK Object Loader");
    mainWindow.resize(1000, 800);

    // Create the VTK widget
    QVTKOpenGLNativeWidget* vtkWidget = new QVTKOpenGLNativeWidget(&mainWindow);

    // Create buttons
    QPushButton* loadFigureButton = new QPushButton("Load Figure", &mainWindow);
    loadFigureButton->setFixedSize(150, 40);

    QPushButton* QuitButton = new QPushButton("Quit", &mainWindow);
    QuitButton->setFixedSize(150, 40);

    QPushButton* selectFileButton = new QPushButton("Select File", &mainWindow);
    selectFileButton->setFixedSize(150, 40);

    QPushButton* RadiusEstimButton = new QPushButton("Estimate Radius", &mainWindow);
    RadiusEstimButton->setFixedSize(150, 40);

    // Create a main layout
    QVBoxLayout* mainLayout = new QVBoxLayout(&mainWindow);

    // Create a vertical layout for the buttons
    QVBoxLayout* buttonLayout = new QVBoxLayout();
    buttonLayout->addWidget(selectFileButton);
    buttonLayout->addWidget(loadFigureButton);
    buttonLayout->addWidget(RadiusEstimButton);
    buttonLayout->addWidget(QuitButton);

    buttonLayout->addStretch();

    // Create a horizontal layout for the button layout and VTK widget
    QHBoxLayout* horizontalLayout = new QHBoxLayout();
    horizontalLayout->addLayout(buttonLayout);
    horizontalLayout->addWidget(vtkWidget);

    // Add the horizontal layout to the main layout
    mainLayout->addLayout(horizontalLayout);

    // Create a renderer
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(colors->GetColor3d("BkgColor").GetData());

    // Create the render window (using the correct type)
    // vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    renderWindow->SetSize(100, 100);

    vtkWidget->setRenderWindow(renderWindow);

    QString fileName = "";

    QObject::connect(loadFigureButton, &QPushButton::clicked, [&]() {
      if (!fileName.isEmpty()) {
        loadObject(fileName, renderer.Get(), renderWindow.Get());
      }
      else {
        std::cerr << "Invalid file argument\n";
      }

    });

    QObject::connect(QuitButton, &QPushButton::clicked, [&]() {
      mainWindow.close();
    });

    QObject::connect(RadiusEstimButton, &QPushButton::clicked, [&]() {
      if (isLoaded)
        calculateRadius();
      else
        std::cerr << "PCD file initialization error...\n";
    });

    QObject::connect(selectFileButton, &QPushButton::clicked, [&]() {
      // Open the file selection dialog
      if (fileName.isEmpty())
        fileName = QFileDialog::getOpenFileName(&mainWindow, QFileDialog::tr("Open PCD File"), home_dir.c_str(), QFileDialog::tr("PCD Files (*.pcd)"));

      // Check if file is selected
      if (!fileName.isEmpty()) {
          std::cout << "Selected file: " << fileName.toStdString() << '\n';
          // Add your file handling logic here
      }
      else {
        std::cerr << "Invalid file argument\n";
      }
    });


    // Show the main window
    mainWindow.show();

    // Start the Qt event loop
    return app.exec();
}
