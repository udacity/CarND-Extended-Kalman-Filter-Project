import argparse
import base64
from datetime import datetime
import os
import shutil
import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from flask import Flask
from io import BytesIO
from subprocess import call
from subprocess import check_output
import csv

sio = socketio.Server()
app = Flask(__name__)
model = None

def get_last_row(csv_filename):
	with open(csv_filename) as tsv:
		lastrow = None
		for lastrow in csv.reader(tsv, dialect="excel-tab"): pass
		return lastrow


@sio.on('telemetry')
def telemetry(sid, data):
	if data:

		output = check_output([model,"data_in.txt","data_out.txt"])

		lastrow = get_last_row("data_out.txt")
		
		x_markers = lastrow[0]
		y_markers = lastrow[1]

		#print("Sending: "+x_markers+" , "+y_markers)

		output = output.decode("utf-8") 
		#print(output)
		outputVals = output.split('\n')
		
		if outputVals[0].find("RMSE") != -1:
			send_estimate_rmse(x_markers,y_markers, outputVals[1],outputVals[2],outputVals[3],outputVals[4])
		else:	
			send_estimate(x_markers,y_markers)
	else:
		# NOTE: DON'T EDIT THIS.
		sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)


def send_estimate(estimate_x, estimate_y):
    sio.emit(
        "estimate",
        data={
            'estimate_x': estimate_x,
            'estimate_y': estimate_y
        },
        skip_sid=True)

def send_estimate_rmse(estimate_x, estimate_y, rmse_x, rmse_y, rmse_vx, rmse_vy):
    sio.emit(
        "estimate_rmse",
        data={
            'estimate_x': estimate_x,
            'estimate_y': estimate_y,
            'rmse_x': rmse_x,
            'rmse_y': rmse_y,
            'rmse_vx': rmse_vx,
            'rmse_vy': rmse_vy
        },
        skip_sid=True)

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Kalman Filter Tracker')
	parser.add_argument(
		'model',
		type=str,
		help='Path to compiled c++ kalam filter file'
	)

	args = parser.parse_args()
	model = args.model

	# wrap Flask application with engineio's middleware
	app = socketio.Middleware(sio, app)

	# deploy as an eventlet WSGI server
	eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
