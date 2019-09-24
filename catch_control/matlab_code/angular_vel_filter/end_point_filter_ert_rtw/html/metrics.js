function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.fcn["MdnFltH15_M_BSymm_uD_yD"] = {file: "/home/zd/project/catch_bottle/catch_control/matlab_code/angular_vel_filter/end_point_filter_ert_rtw/end_point_filter.cpp",
	stack: 396,
	stackTotal: 412};
	 this.metricsArray.fcn["PadBSymm_uD"] = {file: "/home/zd/project/catch_bottle/catch_control/matlab_code/angular_vel_filter/end_point_filter_ert_rtw/end_point_filter.cpp",
	stack: 16,
	stackTotal: 16};
	 this.metricsArray.fcn["end_poing_filter::end_poing_filter"] = {file: "/home/zd/project/catch_bottle/catch_control/matlab_code/angular_vel_filter/end_point_filter_ert_rtw/end_point_filter.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["end_poing_filter::getRTM"] = {file: "/home/zd/project/catch_bottle/catch_control/matlab_code/angular_vel_filter/end_point_filter_ert_rtw/end_point_filter.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["end_poing_filter::initialize"] = {file: "/home/zd/project/catch_bottle/catch_control/matlab_code/angular_vel_filter/end_point_filter_ert_rtw/end_point_filter.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["end_poing_filter::step"] = {file: "/home/zd/project/catch_bottle/catch_control/matlab_code/angular_vel_filter/end_point_filter_ert_rtw/end_point_filter.cpp",
	stack: 152,
	stackTotal: 564};
	 this.metricsArray.fcn["end_poing_filter::terminate"] = {file: "/home/zd/project/catch_bottle/catch_control/matlab_code/angular_vel_filter/end_point_filter_ert_rtw/end_point_filter.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["end_poing_filter::~end_poing_filter"] = {file: "/home/zd/project/catch_bottle/catch_control/matlab_code/angular_vel_filter/end_point_filter_ert_rtw/end_point_filter.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["memset"] = {file: "/home/zd/matlab/sys/lcc/include/string.h",
	stack: 0,
	stackTotal: 0};
	 this.getMetrics = function(token) { 
		 var data;
		 data = this.metricsArray.var[token];
		 if (!data) {
			 data = this.metricsArray.fcn[token];
			 if (data) data.type = "fcn";
		 } else { 
			 data.type = "var";
		 }
	 return data;}
}
	 CodeMetrics.instance = new CodeMetrics();
