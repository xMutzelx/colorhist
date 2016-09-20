#!/usr/bin/env python
"""
Module defining the ColorHist trainer to get the color histogram
"""

from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward
from object_recognition_core.db import Document, Documents
from ecto_opencv import calib, features2d, highgui
from ecto_opencv.features2d import FeatureDescriptor
from object_recognition_core.ecto_cells.db import ModelWriter
from object_recognition_core.pipelines.training import TrainerBase
from object_recognition_colorhist import colorhist_training
import ecto
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward

########################################################################################################################

class ColorHistTrainer(ecto.BlackBox, TrainerBase):
    def __init__(self, *args, **kwargs):
        ecto.BlackBox.__init__(self, *args, **kwargs)
        TrainerBase.__init__(self)

    @classmethod
    def declare_cells(cls, _p):
        # passthrough cells
        cells = {'object_id': CellInfo(ecto.Constant),
                 'json_db': CellInfo(ecto.Constant)}

        # 'real cells'
        cells.update({'model_filler': colorhist_training.ModelFiller(),
                      'model_writer': CellInfo(ModelWriter, params={'method':'ColorHist'}),
                      'trainer': CellInfo(colorhist_training.Trainer)})

        return cells

    @classmethod
    def declare_forwards(cls, _p):
        p = {'json_db': [Forward('value', 'json_db')],
             'object_id': [Forward('value', 'object_id')]}
        p.update({'trainer': 'all'})
        i = {}
        o = {}

        return (p,i,o)

    def connections(self, p):
        connections = [ self.object_id[:] >> self.trainer['object_id'],
                        self.json_db[:] >> self.trainer['json_db'] ]
        connections += [ self.trainer['colorValues'] >> self.model_filler['colorValues'] ]
        connections += [ self.trainer['histogram'] >> self.model_filler['histogram'] ]
        
        # Connect the model builder to the source
        connections += [ self.object_id[:] >> self.model_writer['object_id'],
                         self.json_db[:] >> self.model_writer['json_db'],
                         self.model_filler['db_document'] >> self.model_writer['db_document']]

        return connections
