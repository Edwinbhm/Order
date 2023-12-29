import json

with open('order.json','r') as file:
    data = json.load(file)

print(data)