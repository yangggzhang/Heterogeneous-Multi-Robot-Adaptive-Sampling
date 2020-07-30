import rosbag
import matplotlib.pyplot as plt
import numpy as np
# plt.rc('text', usetex=True)
# plt.rc('font', family='serif')
# from sampling_msg.msg import report


# obstacle_id = "obs_2_"
# rosbag_file = "hetero_temp.bag"
experiment_num = 2

def get_num_samples_from_data(bag_name, length, scenario_num):
	total_rms = []
	for i in range(scenario_num):
		for j in range(experiment_num):
			rosbag_file = bag_name+"scenario"+str(i)+"_"+str(j)+".bag"
			rms = []
			bag  = rosbag.Bag(rosbag_file)
			for topic, msg, t in bag.read_messages(topics=['/sampling_performance']):
				rms.append(msg.num_samples)
			if not (len(rms)<100):
				total_rms.append(rms[:length])
				print(len(rms))
	total_rms = np.array(total_rms)
	return total_rms

def get_rmse_from_data(bag_name, length, scenario_num):
	total_rms = []
	for i in range(scenario_num):
		for j in range(experiment_num):
			rosbag_file = bag_name+"scenario"+str(i)+"_"+str(j)+".bag"
			rms = []
			bag  = rosbag.Bag(rosbag_file)
			for topic, msg, t in bag.read_messages(topics=['/sampling_performance']):
				rms.append(msg.rms)
			if not (len(rms)<100):
				total_rms.append(rms[:length])
				print(len(rms))
	total_rms = np.array(total_rms)
	return total_rms

def get_rmse_from_data_free(bag_name, length, obstacle_name):
	total_rms = []
	for j in range(experiment_num):
		rosbag_file = bag_name+"scenario"+str(scenario)+"_"+str(j)+".bag"
		rms = []
		bag  = rosbag.Bag(rosbag_file)
		for topic, msg, t in bag.read_messages(topics=['/sampling_performance']):
			rms.append(msg.rms)
		if not (len(rms)<90):
			total_rms.append(rms[:length])
			print(len(rms))
	total_rms = np.array(total_rms)
	return total_rms

def bag_info_to_list(bag, info):
	info_list=[]
	for topic, msg, t in bag.read_messages(topics=['/sampling_performance']):
		if info == "sample_count":
			info_list.append(msg.sample_count)
		elif info == "rmse":
			info_list.append(msg.rmse)
		elif info == "average_variance":
			info_list.append(msg.average_variance)
		else:
			raise NameError('msg name not correct. Try sample_count, rmse, average_variance')
	print(info_list)
	return info_list

def get_info_from_data_by_type_all_experiments(bag_name, length, scenario_num, info_type):
	total_info = []
	for i in range(1,scenario_num+1): #scenario 1 indexed
		for j in range(experiment_num):
			rosbag_file = bag_name+"scenario"+str(i)+"_"+str(j)+".bag"
			bag  = rosbag.Bag(rosbag_file)
			info_list = bag_info_to_list(bag, info_type)
			if not (len(info_list)<length):
				total_info.append(info_list[:length])
				print(len(info_list))
	total_info = np.array(total_info)
	return total_info

def get_info_from_data_by_type_single_scenario(bag_name, length, scenario_id, info_type):
	total_info = []
	for j in range(experiment_num):
		rosbag_file = bag_name+"scenario"+str(scenario_id)+"_"+str(j)+".bag"
		bag  = rosbag.Bag(rosbag_file)
		info_list = bag_info_to_list(bag, info_type)
		if not (len(info_list)<length):
			total_info.append(info_list[:length])
			print(len(info_list))
	total_info = np.array(total_info)
	print(total_info.shape)
	return total_info
	
def main():
	#----------------------------------------#
	# Get mean and std over all experiments  #
	#----------------------------------------#

	# obstacle_config_num = 2
	# homo_total_rms = get_info_from_data_by_type_all_experiments("./homo/", 10, obstacle_config_num, 'rmse')
	# print(homo_total_rms.shape)
	# homo_mean_rms = homo_total_rms.mean(axis=0)
	# homo_std_rms = homo_total_rms.std(axis=0)
	# homo_t = np.arange(homo_total_rms.shape[1])
	# print(homo_t.shape)
	# print(homo_mean_rms.shape)

	obstacle_config_num = 1
	hetero_total_rms = get_info_from_data_by_type_all_experiments("./hetero/", 10, obstacle_config_num, 'rmse')
	print(hetero_total_rms.shape)
	hetero_mean_rms = hetero_total_rms.mean(axis=0)
	hetero_std_rms = hetero_total_rms.std(axis=0)
	hetero_t = np.arange(hetero_total_rms.shape[1])
	print(hetero_t.shape)
	print(hetero_mean_rms.shape)

	# obstacle_config_num = 2
	# homo_total_num_samples = get_info_from_data_by_type_all_experiments("./homo/", 10, obstacle_config_num, 'sample_count')
	# print(homo_total_num_samples.shape)
	# homo_mean_num_samples = homo_total_num_samples.mean(axis=0)
	# homo_std_num_samples = homo_total_num_samples.std(axis=0)
	# homo_t = np.arange(homo_total_num_samples.shape[1])
	# print(homo_t.shape)
	# print(homo_mean_num_samples.shape)

	obstacle_config_num = 1
	hetero_total_num_samples = get_info_from_data_by_type_all_experiments("./hetero/", 10, obstacle_config_num, 'sample_count')
	print(hetero_total_num_samples.shape)
	hetero_mean_num_samples = hetero_total_num_samples.mean(axis=0)
	hetero_std_num_samples = hetero_total_num_samples.std(axis=0)
	hetero_t = np.arange(hetero_total_num_samples.shape[1])
	print(hetero_t.shape)
	# print(hetero_mean_num_samples.shape)

	# obstacle_config_num = 2
	# homo_total_avg_variance = get_info_from_data_by_type_all_experiments("./homo/", 10, obstacle_config_num, 'average_variance')
	# print(homo_total_avg_variance.shape)
	# homo_mean_avg_variance = homo_total_num_samples.mean(axis=0)
	# homo_std_avg_variance = homo_total_num_samples.std(axis=0)
	# homo_t = np.arange(homo_total_avg_variance.shape[1])
	# print(homo_t.shape)
	# print(homo_mean_avg_variance.shape)

	obstacle_config_num = 1
	hetero_total_avg_variance = get_info_from_data_by_type_all_experiments("./hetero/", 10, obstacle_config_num, 'average_variance')
	print(hetero_total_avg_variance.shape)
	hetero_mean_avg_variance = hetero_total_avg_variance.mean(axis=0)
	hetero_std_avg_variance = hetero_total_avg_variance.std(axis=0)
	hetero_t = np.arange(hetero_total_avg_variance.shape[1])
	print(hetero_t.shape)
	# print(hetero_mean_num_samples.shape)

	#------------------------------------#
	# Code for checking single scenario  #
	#------------------------------------#
	# scenario_id = 1
	# homo_total_rms = get_info_from_data_by_type_single_scenario("./homo/", 10, scenario_id=scenario_id, info_type='rmse')
	# print(homo_total_rms.shape)
	# homo_mean_rms = homo_total_rms.mean(axis=0)
	# homo_std_rms = homo_total_rms.std(axis=0)
	# homo_t = np.arange(homo_total_rms.shape[1])
	# print(homo_t.shape)
	# print(homo_mean_rms.shape)

	# scenario_id = 1
	# hetero_total_rms = get_info_from_data_by_type_single_scenario("./hetero/", 10, scenario_id=scenario_id, info_type='rmse')
	# print(hetero_total_rms.shape)
	# hetero_mean_rms = hetero_total_rms.mean(axis=0)
	# hetero_std_rms = hetero_total_rms.std(axis=0)
	# hetero_t = np.arange(hetero_total_rms.shape[1])
	# print(hetero_t.shape)
	# print(hetero_mean_rms.shape)

	#------------------------------------#
	#             Plot Result            #
	#------------------------------------#	
	plt.rcParams.update({'font.size': 30})
	plt.rcParams['pdf.fonttype']=42
	fig, ax = plt.subplots(1)
	# line1 = ax.plot(homo_t, homo_mean_rms,lw=2, label="homogeneous RMS error", color = 'blue')
	# ax.fill_between(homo_t, homo_mean_rms+homo_std_rms, homo_mean_rms-homo_std_rms, facecolor='blue',alpha=0.1)

	line2 = ax.plot(hetero_t, hetero_mean_rms,lw=2, label="heterogeneous RMS error", color = 'red')
	ax.fill_between(hetero_t, hetero_mean_rms+hetero_std_rms, hetero_mean_rms-hetero_std_rms, facecolor='red',alpha=0.1)
	ax.set_ylabel('RMS Error (uW)', size=35)
	ax.set_xlabel('ROS Time (s)',size=35)
	ax.legend(loc=0)
	ax2 = ax.twinx()
	# line3 = ax2.plot(homo_t, homo_mean_num_samples,lw=2, label="homogeneous number of samples", color = 'blue',linestyle="--")
	# ax2.fill_between(homo_t, homo_mean_num_samples+homo_std_num_samples, homo_mean_num_samples-homo_std_num_samples, facecolor='blue',alpha=0.1)
	plt.axvline(x=25, color="maroon", lw=2.5)
	plt.text(15, -33, 'Ariel Robot \nBattery Died',size=26)
	line4 = ax2.plot(hetero_t, hetero_mean_num_samples,lw=2, label="heterogeneous number of samples", color = 'red',linestyle="--")
	ax2.fill_between(hetero_t, hetero_mean_num_samples+hetero_std_num_samples, hetero_mean_num_samples-hetero_std_num_samples, facecolor='red',alpha=0.1)
	# lines = line1 + line2 + line3 + line4
	ax2.set_xlabel('ROS Time (s)',size=35)
	ax2.set_ylabel('Number of Samples',size=35)
	# ax.legend(lines, [l.get_label() for l in lines], loc=9, prop={'size': 26})

	plt.show()


if __name__ == '__main__':
	main()