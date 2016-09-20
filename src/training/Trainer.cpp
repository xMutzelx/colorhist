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

#include "training.h"
#include "visualizer.h"

using namespace cv;
using namespace std;

/* cell storing the 3d points and descriptors while a model is being computed */
struct Trainer
{
  public:
    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      inputs.declare(&Trainer::json_db_, "json_db", "The parameters of the DB as a JSON string.").required(true);
      inputs.declare(&Trainer::object_id_, "object_id", "The id of the object in the DB.").required(true);

      outputs.declare(&Trainer::trainer_, "colorValues", "Color values in a matrix.");
      outputs.declare(&Trainer::visualizer_, "histogram", "The computed color histogram.");
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {
      /* Get the DB */
      object_recognition_core::db::ObjectDbPtr db =
        object_recognition_core::db::ObjectDbParameters(*json_db_).generateDb();
      object_recognition_core::db::Documents documents =
        object_recognition_core::db::ModelDocuments(db,
            std::vector<object_recognition_core::db::ObjectId>(1, *object_id_),
            "mesh");
      if (documents.empty())
      {
        std::cerr << "Skipping object id \"" << *object_id_ << "\" : no mesh in the DB" << std::endl;
        return ecto::OK;
      }

      /* Get the list of _attachments and figure out the original one */
      object_recognition_core::db::Document document = documents[0];
      std::vector<std::string> attachments_names = document.attachment_names();
      std::string mesh_path;
      std::string possible_name = "cloud";

      BOOST_FOREACH(const std::string& attachment_name, attachments_names)
      {
        if (attachment_name.find(possible_name) != 0)
        {
          continue;
        }

        /* Create a temporary file */
        char mesh_path_tmp[L_tmpnam];
        tmpnam(mesh_path_tmp);
        mesh_path = std::string(mesh_path_tmp) + attachment_name.substr(possible_name.size());

        /* Load the mesh and save it to the temporary file */
        std::ofstream in_file;
        in_file.open(mesh_path.c_str());
        document.get_attachment_stream(attachment_name, in_file);
        in_file.close();
      }

      /* Skip if no cloud.ply was found */
      if(mesh_path.empty())
      {
        std::cerr << "Skipping object id \"" << *object_id_ << "\" : no cloud in the DB" << std::endl;
        return ecto::OK;
      }
      
      int skip_header_lines = 0;
      int itoken = 1;
      int avg_color[3] = {0,0,0};
      int doc_length = 0;
      cv::Vec3f T;
      std::vector<boost::array<int, 3> > color_list;

      /*
       * Opens cloud.ply and read it line by line
       * Every entry is read seperatly
       * Every color entry is safed in Colorhist-Trainer
      */
      std::ifstream out_file;
      std::string line;
      std::string token;
      out_file.open(mesh_path.c_str());

      while(std::getline(out_file, line))
      {
        std::istringstream iss(line);

		/* Read the document length and resize the list */
        if(skip_header_lines == 2)
        {
          for(int i=0; i<3; i++)
          {
            iss >> token;
            itoken = atoi(token.c_str());

            if(i==2)
            {
              doc_length = itoken;
              color_list.resize(doc_length);
            }
          }
        }

        /* Skip the first 12 lines to get the relevant number of entries in this file */
        if(skip_header_lines >= 13)
        {
          for(int i=0; i<6; i++)
          {
            iss >> token;
            itoken = atoi(token.c_str());

            if(i == 3)
              color_list[skip_header_lines-13][0]=scale_color(itoken);
            if(i == 4)
              color_list[skip_header_lines-13][1]=scale_color(itoken);
            if(i == 5)
              color_list[skip_header_lines-13][2]=scale_color(itoken);
          }

          /* Calculate the average color */
          avg_color[0]=get_avg_color(color_list[skip_header_lines-13][0], avg_color[0], false, skip_header_lines-13);
          avg_color[1]=get_avg_color(color_list[skip_header_lines-13][1], avg_color[1], false, skip_header_lines-13);
          avg_color[2]=get_avg_color(color_list[skip_header_lines-13][2], avg_color[2], false, skip_header_lines-13);
        }
        skip_header_lines++;
      }

      /*
       * Create the color histogram
       *
       * color_list[x][0] = red
       * color_list[x][1] = green
       * color_list[x][2] = blue
       *
       * 0 - 255 color values are scaled in steps by 10
       * => 26 steps
       * 2-dimensional array[26][3]
      */

      float colorhist_array[26][3];
      for(int i=0; i<26; i++)
      {
        for(int j=0; j<3; j++)
        {
          colorhist_array[i][j]=0;
        }
      }
		
	  /* skip_header_lines-14 because skip_header_lines is incremented one more time */
      for(int i=0; i<skip_header_lines-14; i++)
      {
        colorhist_array[(color_list[i][0]/10)][0]++;
        colorhist_array[(color_list[i][1]/10)][1]++;
        colorhist_array[(color_list[i][2]/10)][2]++;
      }

      /* Calculate the percentage of the color values */
      for(int i=0; i<26; i++)
      {
        colorhist_array[i][0] = (colorhist_array[i][0]*100)/doc_length;
        colorhist_array[i][1] = (colorhist_array[i][1]*100)/doc_length;
        colorhist_array[i][2] = (colorhist_array[i][2]*100)/doc_length;

        T[0]=colorhist_array[i][0];
        T[1]=colorhist_array[i][1];
        T[2]=colorhist_array[i][2];

        trainer_->push_back(cv::Mat(T));
      }

	  /* Calculate the avg. color values */
	  for(int i=0; i<3; i++)
	  {
		avg_color[i]=get_avg_color(color_list[skip_header_lines-14][i], avg_color[i], true, skip_header_lines-13);
		T[i]=avg_color[i];
      }

      trainer_->push_back(cv::Mat(T));

      out_file.close();

      /* Visualize the histogram and draw the average color */
      *visualizer_ = visualizeHistogram(colorhist_array, avg_color);

      return ecto::OK;
    }

  private:
    ecto::spore<std::string> json_feature_params_;
    ecto::spore<std::string> json_descriptor_params_;
    ecto::spore<std::string> object_id_;
    ecto::spore<std::string> json_db_;
    ecto::spore<std::vector<cv::Mat> > trainer_;
    ecto::spore<cv::Mat> visualizer_;
};

ECTO_CELL(colorhist_training, Trainer, "Trainer",
          "Compute ColorHist models for a given object")
