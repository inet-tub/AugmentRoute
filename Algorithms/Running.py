import pandas as pd
import CombinedAlgorithms as Alg
import numpy as np


input = np.load("1Waypoint-1.1-100Nodes-250Pairs.npy", allow_pickle=True)
toExport = Alg.runForAllGraphs(input)

df = pd.DataFrame(toExport)
writer = pd.ExcelWriter('Test.xlsx', engine='xlsxwriter')
df.to_excel(writer, sheet_name='Combined', index=False)
writer.save()
