function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.fcn["ForceInteraction::ForceInteraction"] = {file: "/home/gx/Projects/match/catch_control/matlab_code/force_interaction/force_interaction_ert_rtw/force_interaction.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceInteraction::getRTM"] = {file: "/home/gx/Projects/match/catch_control/matlab_code/force_interaction/force_interaction_ert_rtw/force_interaction.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceInteraction::initialize"] = {file: "/home/gx/Projects/match/catch_control/matlab_code/force_interaction/force_interaction_ert_rtw/force_interaction.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceInteraction::step"] = {file: "/home/gx/Projects/match/catch_control/matlab_code/force_interaction/force_interaction_ert_rtw/force_interaction.cpp",
	stack: 224,
	stackTotal: 224};
	 this.metricsArray.fcn["ForceInteraction::terminate"] = {file: "/home/gx/Projects/match/catch_control/matlab_code/force_interaction/force_interaction_ert_rtw/force_interaction.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["ForceInteraction::~ForceInteraction"] = {file: "/home/gx/Projects/match/catch_control/matlab_code/force_interaction/force_interaction_ert_rtw/force_interaction.cpp",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["memcpy"] = {file: "/home/gx/Matlab/sys/lcc/include/string.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["memset"] = {file: "/home/gx/Matlab/sys/lcc/include/string.h",
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
