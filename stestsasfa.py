import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import mpld3
from mpld3 import plugins


df = pd.DataFrame.from_dict({'A':[1,2,3],
                    'B':[1,2,3,5,6],
                    'C':[1,3],}, orient='index')

label = df.T
label = str(label.to_html())
print(label)
