#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <boost/array.hpp>
#include <boost/foreach.hpp>

#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/rgbd/rgbd.hpp>
#include <opencv2/opencv.hpp>

#include <object_recognition_core/common/types_eigen.h>
#include <object_recognition_core/common/json.hpp>
#include <object_recognition_core/db/db.h>
#include <object_recognition_core/db/document.h>
#include <object_recognition_core/db/model_utils.h>
#include <object_recognition_core/db/view.h>
#include <object_recognition_core/db/opencv.h>

#include <object_recognition_renderer/renderer3d.h>
#include <object_recognition_renderer/utils.h>

#include "db_colorhist.h"

using namespace cv;
using namespace std;

/* cell storing the 3d points and descriptors while a model is being computed */
struct ModelReader
{
  public:
    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare(&ModelReader::json_db_, "json_db", "The parameters of the DB as a JSON string.").required(true);
      inputs.declare(&ModelReader::object_ids_, "object_ids", "The ids of the objects in the DB.").required(true);

	  outputs.declare(&ModelReader::model_data_, "model_data", "Model id + number of Variations");
      outputs.declare(&ModelReader::model_colorValues_, "model_colorValues", "Color values in a matrix");
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      std::vector<std::string> object_ids;
      object_ids = *object_ids_;
      std::vector<std::string> model_data;

      std::vector<cv::Mat> detected_model_colorValues;

      std::vector<cv::Mat> model_colorValues;
      float model_colorhist_array[26][3];
      std::fill(model_colorhist_array[0], model_colorhist_array[0] + 26 * 3, 0);
      for(unsigned int i=0; i<object_ids.size(); i++)
      {  
        /* Get the DB */
        object_recognition_core::db::ObjectDbPtr db =
          object_recognition_core::db::ObjectDbParameters(*json_db_).generateDb();
        object_recognition_core::db::Documents documents =
          object_recognition_core::db::ModelDocuments(db,
              std::vector<object_recognition_core::db::ObjectId>(1, object_ids[i] ), "ColorHist");

        detected_model_colorValues.clear();
        if (documents.empty())
        {
          std::cerr << "Skipping object id \"" << object_ids[i] << "\" : no mesh in the DB" << std::endl;
          return ecto::OK;
        }

        /* Get the list of _attachments and figure out the original one */
        object_recognition_core::db::Document document = documents[0];
        std::vector<std::string> attachments_names = document.attachment_names();
        std::string possible_name = "colorValues";

        BOOST_FOREACH(const std::string& attachment_name, attachments_names)
        {
          if (attachment_name.find(possible_name) != 0)
          {
            continue;
          }

          document.get_attachment<std::vector<cv::Mat> >(attachment_name, detected_model_colorValues);
        }

        /* Check if the recognized object has color information in the database */
        if(detected_model_colorValues.empty())
        {
          std::cerr << "This object (" << object_ids[i] << ") : contains no relevant data in the DB" << std::endl;
          model_colorhist_array[0][0] = 2;
        }
        else
        {
          /* Safe the colorvalues from the database into an array
           * Datatype: std::vector<cv::Mat> => vector = 26, Matrix(1D) = 3
           */
          for(int j= 0; j<26; j++)
          {
            for (int k = 0; k < 3; ++k)
            {
              model_colorhist_array[j][k]=detected_model_colorValues[j].at<float>(0,k);
            }
          }
        }
        /* Safe the array into an 2D Matrix
         * Datatype: cv::Mat => Matrix(2D) 26|3
         */
        cv::Mat M= cv::Mat(26,3, CV_32F, model_colorhist_array);

        /* Push the matrix into an vector of marices
         * Datatype: std::vector<cv::Mat> => vector = number of objects, Matrix(2D) = 26|3
         */
        model_colorValues.push_back(cv::Mat(M).clone());
        model_data.push_back(object_ids[i]);
        std::cout << object_ids[i] << std::endl;
        /* If the object has no variations, the counter is zero */
        int counter = 0;

        /* If the object has variations of itself, then repeate everything until
         * all possible variations got checked 
         */
        if(document.has_field("Variations"))
        {
          std::vector<cv::Mat> detected_variations_colorValues;

          float variations_colorhist_array[26][3];
          std::fill(variations_colorhist_array[0], variations_colorhist_array[0] + 26 * 3, 0);

		  /* Filter each ID out of the variation database field */
          std::string ids = document.get_field<string>("Variations");
          counter = std::count(ids.begin(), ids.end(), ';');
          std::string delimiter = ";";
		  std::string variation_id;
          size_t pos = 0;
          
          /* Push the number of variations back */
		  stringstream ss;
		  ss << counter;
		  string str = ss.str();
          model_data.push_back(str);
          
          /* Check for new ID's as long as there are ";" at the end */
          while ((pos = ids.find(delimiter)) != std::string::npos) 
          {
            variation_id = ids.substr(0, pos);
            ids.erase(0, pos + delimiter.length());

            object_recognition_core::db::ObjectDbPtr db2 =
              object_recognition_core::db::ObjectDbParameters(*json_db_).generateDb();
            object_recognition_core::db::Documents documents2 =
              object_recognition_core::db::ModelDocuments(db2,
                  std::vector<object_recognition_core::db::ObjectId>(1, variation_id ), "ColorHist");

			model_data.push_back(variation_id);

            detected_variations_colorValues.clear();
            if (documents.empty())
            {
              std::cerr << "Skipping object id \"" << variation_id << "\" : no mesh in the DB" << std::endl;
              return ecto::OK;
            }

            /* Get the list of _attachments and figure out the original one */
            object_recognition_core::db::Document document2 = documents2[0];
            attachments_names = document2.attachment_names();

            BOOST_FOREACH(const std::string& attachment_name, attachments_names)
            {
              if (attachment_name.find(possible_name) != 0)
              {
                continue;
              }

              document2.get_attachment<std::vector<cv::Mat> >(attachment_name, detected_variations_colorValues);
            }

            /* Check if the recognized object has color information in the database */
            if(detected_variations_colorValues.empty())
            {
              std::cerr << "This object (" << variation_id << ") : contains no relevant data in the DB" << std::endl;
              variations_colorhist_array[0][0] = 2;
            }
            else
            {
              /* Safe the colorvalues from the database into an array
               * Datatype: std::vector<cv::Mat> => vector = 26, Matrix(1D) = 3
               */
              for(int j= 0; j<26; j++)
              {
                for (int k = 0; k < 3; ++k)
                {
                  variations_colorhist_array[j][k]=detected_variations_colorValues[j].at<float>(0,k);
                }
              }
            }
            /* Safe the array into an 2D Matrix
             * Datatype: cv::Mat => Matrix(2D) 26|3
             */
            cv::Mat M= cv::Mat(26,3, CV_32F, variations_colorhist_array);

            /* Push the matrix into an vector of marices
             * Datatype: std::vector<cv::Mat> => vector = number of objects, Matrix(2D) = 26|3
             */
            model_colorValues.push_back(cv::Mat(M).clone());
          }
        }
        /* Otherwise safe the number zero for further calculations */ 
        else
        {
			model_data.push_back("0");
		}
      }
      
      *model_colorValues_=model_colorValues;
      model_colorValues.clear();
      
      *model_data_=model_data;
      model_data.clear();

      return ecto::OK;
    }

  private:
    ecto::spore<std::string> json_feature_params_;
    ecto::spore<std::string> json_descriptor_params_;
    ecto::spore<std::vector<std::string> > object_ids_;
    ecto::spore<std::string> json_db_;
    ecto::spore<std::vector<cv::Mat> > model_colorValues_;
    ecto::spore<std::vector<std::string> > model_data_;
};

ECTO_CELL(colorhist_detection, ModelReader, "ModelReader",
          "Compute ColorHist models for a given object")

