#include <ecto/ecto.hpp>

#include <opencv2/core/core.hpp>

#include <object_recognition_core/db/document.h>
#include <object_recognition_core/db/document.h>
#include <object_recognition_core/db/opencv.h>
#include <object_recognition_core/db/prototypes/observations.hpp>
#include <object_recognition_core/common/types.h>

#include "db_colorhist.h"

using object_recognition_core::db::ObjectId;
using object_recognition_core::db::Document;

using namespace cv;
using namespace std;

/* Fills the database with one histogram.png and the colorValues */
namespace colorhist
{
  struct ModelFiller
  {
  public:
    static void declare_io(const ecto::tendrils& params, ecto::tendrils& inputs, ecto::tendrils& outputs)
    {
      typedef ModelFiller C;
      inputs.declare(&C::trainer_, "colorValues", "Color values in a matrix.");
      inputs.declare(&C::visualizer_, "histogram", "The computed color histogram.");
      outputs.declare(&C::db_document_, "db_document", "The filled document.");
    }

    int process(const ecto::tendrils& inputs, const ecto::tendrils& outputs)
    {		
		Document db_document;
		std::vector<cv::Mat>  valueCheck;
		cv::Mat imageCheck;
		
		valueCheck = *trainer_;
		imageCheck = *visualizer_;
		
		if(!valueCheck.empty())
			db_document.set_attachment<std::vector<cv::Mat> >("colorValues", *trainer_);

		if(!imageCheck.empty())
			object_recognition_core::db::png_attach(*visualizer_, db_document, "histogram");		

		*db_document_ = db_document;

		return ecto::OK;
    }

  private:
    ecto::spore<object_recognition_core::db::Document> db_document_;
    ecto::spore<std::vector<cv::Mat> > trainer_;
    ecto::spore<cv::Mat> visualizer_;
  };
}

ECTO_CELL(colorhist_training, colorhist::ModelFiller, "ModelFiller",
    "Populates a db document with a ColorHist model for persisting a later date.")
