from datetime import datetime



now = datetime.now()
dt_string = now.strftime("%d_%m_%Y_%H:%M:%S")

nueva = dt_string.replace('_','-',2)
nueva = nueva.replace('_',' ',1)
print(dt_string)
print(nueva)
