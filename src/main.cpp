#include <cstdlib>

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
#include <pcl/point_types.h>

void createVTKObject(QString &fileName, vtkRenderer* renderer, vtkGenericOpenGLRenderWindow* renderWindow) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

  if (pcl::io::loadPCDFile(fileName.toStdString(), *cloud) == -1) {
    // PCL_ERROR("Couldn't read PCD file!\n");
    std::exit(EXIT_FAILURE);
  }
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
    loadFigureButton->setFixedSize(100, 30);  // Set the desired width and height
    QPushButton* QuitButton = new QPushButton("Quit", &mainWindow);
    QuitButton->setFixedSize(100, 30);
    QPushButton* selectFileButton = new QPushButton("Select File", &mainWindow);
    selectFileButton->setFixedSize(100, 30);
    // QPushButton* QuitButton = new QPushButton("Quit", &mainWindow);
    // QuitButton->setFixedSize(100, 30);

    // Create a main layout
    QVBoxLayout* mainLayout = new QVBoxLayout(&mainWindow);

    // Create a vertical layout for the buttons
    QVBoxLayout* buttonLayout = new QVBoxLayout();
    buttonLayout->addWidget(loadFigureButton);
    buttonLayout->addWidget(selectFileButton);
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
        createVTKObject(fileName, renderer.Get(), renderWindow.Get());
      }
      else {
        std::cerr << "Invalid file argument\n";
      }

    });
     QObject::connect(QuitButton, &QPushButton::clicked, [&]() {
      mainWindow.close();
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
