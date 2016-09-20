include_directories(${CMAKE_CURRENT_SOURCE_DIR}/common)

# deal with the training colorhist modules
ectomodule(colorhist_training DESTINATION ${PROJECT_NAME}
                         INSTALL
                         training/ModelFiller.cpp
                         training/module.cpp
                         training/Trainer.cpp
                         training/training.cpp
			 common/db_colorhist.cpp
			 common/visualizer.cpp
)

link_ecto(colorhist_training ${catkin_LIBRARIES}
                             ${OpenCV_LIBRARIES}
)

# deal with the detection colorhist module
ectomodule(colorhist_detection DESTINATION ${PROJECT_NAME}
                          INSTALL  
                          detection/Detector.cpp
                          detection/ModelReader.cpp
                          detection/module.cpp
			  detection/detection.cpp
                          detection/filtering.cpp
                          common/db_colorhist.cpp
)

link_ecto(colorhist_detection ${OpenCV_LIBRARIES}
                              ${catkin_LIBRARIES} 
)
