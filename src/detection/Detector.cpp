#include <string>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <ecto/ecto.hpp>

#include <object_recognition_core/common/types.h>
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/common/pose_result.h>
#include <object_recognition_core/db/opencv.h>

#include "db_colorhist.h"
#include "detection.h"
#include "filtering.h"

using ecto::tendrils;
using object_recognition_core::common::PoseResult;

namespace colorhist
{
  /* Cell that loads a colorhist model from the DB */
  struct Detector
  {
    public:
      static void declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        /* Model = Database model | Object = recognized object */
        inputs.declare(&Detector::json_db_, "json_db", "The parameters of the DB as a JSON string.").required(true);
        inputs.declare(&Detector::model_colorValues_, "model_colorValues", "Color values of the database models.");
        inputs.declare(&Detector::model_data_, "model_data", "Model ids and number of variations");
        inputs.declare(&Detector::object_colorValues_, "object_colorValues", "Color values of the recognized objects");
        inputs.declare(&Detector::pose_results_in_, "pose_results_in", "The results of object recognition");

        outputs.declare(&Detector::pose_results_, "pose_results", "The results of object recognition");
      }

      int process(const tendrils& inputs, const tendrils& outputs)
      {
		object_recognition_core::db::ObjectDbPtr db =
          object_recognition_core::db::ObjectDbParameters(*json_db_).generateDb();  
		  
        std::vector<PoseResult> raw_pose_results;
        PoseResult pose_result;
        raw_pose_results = *pose_results_in_;
        pose_results_->clear();
        std::vector<cv::Mat>  object_colorValues;
        std::vector<cv::Mat>  model_colorValues;
        std::vector<std::string> model_data;
        object_colorValues = *object_colorValues_;
        model_colorValues = *model_colorValues_;
        model_data = *model_data_;

        float model_colorhist_array[26][3];
        std::fill(model_colorhist_array[0], model_colorhist_array[0] + 26 * 3, 0);
        float object_colorhist_array[26][3];
        std::fill(object_colorhist_array[0], object_colorhist_array[0] + 26 * 3, 0);
        double raw_result = 0;
        double conf_result = 0;
        double minConf = 40;
        cv::MatND object_colorhist;
        cv::MatND model_colorhist;

        std::cout<<"object_colorValues: "<<object_colorValues.size()<<std::endl;
        std::cout<<"model_colorValues: "<<model_colorValues.size()<<std::endl;

		/* Check if the detector got data from linemod and the modelreader */
        if(!object_colorValues.empty() && !model_colorValues.empty())
        {
          int pos = 1;
          int check = 1;

          /* Compare the recognized object with all possible models from the database */
          for(unsigned int i=0; i<object_colorValues.size(); i++)
          {
			double best_result = 0;
			std::string best_id = "0";
			
            for(int j = 0; j<=atoi(model_data[pos].c_str()); j++)
            {
              if(model_colorValues[j+pos-check].at<float>(0,0) <=1)
              {
                /* Safe the color values from the object and model into two arrays
                 * colorValues.size() = 27 (model) but the last row contains the avg. colors
                 * and at the moment we dont need the avg. colors => 26 steps
                 * Datatype: cv::Mat(2D) = 26|3 safed into float array[26][3]
                 */
                for(int r= 0; r<26; r++)
                {
                  for (int c = 0; c < 3; ++c)
                  {
                    model_colorhist_array[r][c] = model_colorValues[j+pos-check].at<float>(r,c);
                    object_colorhist_array[r][c] = object_colorValues[i].at<float>(r,c);
                  }
                }

                /* Safe the array into an MatND
                 * The datatype MatND is necessary for cv::compareHist() and it is not
                 * possible to safe cv::Mat into cv::MatND?!?!
                 * Datatype: float array[26][3] into cv::MatND(2D) = 26|3
                 */
                model_colorhist = cv::MatND(26,3, CV_32F, model_colorhist_array);
                object_colorhist = cv::MatND(26,3, CV_32F, object_colorhist_array);

                /* [1|-1] 1 = perfect match, -1 = worst match */
                //~ raw_result = methodCorrelation(object_colorhist, model_colorhist);
                //~ conf_result = calcConf(-1,1,raw_result);
                //~ std::cout<<"1: "<<conf_result<<std::endl;

                /* [0|+infinity] 0 = perfect match, infinity = worst match */
                //~ raw_result = methodChiSquare(object_colorhist, model_colorhist);
                //~ std::cout<<"2 [inf.->0]: "<<raw_result<<std::endl;

                /* [0|1] 0 = worst match, 1 = perfect match (CAUTION: Histograms needs to be normalized) */
                //~ raw_result = methodIntersection(object_colorhist, model_colorhist);
                //~ conf_result = calcConf(0,1,raw_result);
                //~ std::cout<<"3: "<<conf_result<<std::endl;

                /* [0|1] 0 = perfect match, 1 = worst match */
                //~ raw_result = methodBhattacharyya(object_colorhist, model_colorhist);
                //~ conf_result = calcConf(1,0,raw_result);
                //~ std::cout<<"4: "<<conf_result<<std::endl;

                conf_result = meanDecision(object_colorhist, model_colorhist, true, true ,true);

				/* Search the best result among all model-variations */
				if(conf_result>best_result)
				{
					best_result=conf_result;
					if(j==0)
						best_id = model_data[pos-1];
					else
						best_id = model_data[pos+j];
				}
					
                //std::cout<<std::endl;
              }
            }
            
            
            
            std::cout<<best_id<<": "<<best_result<<std::endl;
            raw_pose_results[i].set_confidence(best_result);
            raw_pose_results[i].set_object_id(db, best_id);

            check++;
            pos = pos+atoi(model_data[pos].c_str())+2;
            std::cout<<"-----------------------------------------------------------------------"<<std::endl;
          }
          if(raw_pose_results.size()==0)
			std::cout<<"Empty"<<std::endl;
			
		  
          *pose_results_=raw_pose_results;
        }

        return ecto::OK;
      }

    private:
      ecto::spore<std::vector<object_recognition_core::common::PoseResult> > pose_results_;
      ecto::spore<std::vector<object_recognition_core::common::PoseResult> > pose_results_in_;
      ecto::spore<std::vector<cv::Mat> > object_colorValues_;
      ecto::spore<std::vector<cv::Mat> > model_colorValues_;
      ecto::spore<std::vector<std::string> > model_data_;
      ecto::spore<std::string> json_db_;
  };
}

ECTO_CELL(colorhist_detection, colorhist::Detector, "Detector", "Reads a colorhist model from the db")
