# Modify setup.py file to install the tf-models-official repository targeted at TF v2.8.0
import re

# Read the content of setup.py file
with open('/models/research/object_detection/packages/tf2/setup.py') as f:
    s = f.read()

# Modify the content of setup.py
s = re.sub('tf-models-official>=2.5.1', 'tf-models-official==2.8.0', s)

# Write the modified content back to setup.py
with open('/models/research/setup.py', 'w') as f:
    f.write(s)
