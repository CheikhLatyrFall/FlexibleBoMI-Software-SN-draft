#include "Dislin_Scope.h"

namespace Control_IMU_JACO
{
	Dislin_Scope::Dislin_Scope()
	{}

	Dislin_Scope::~Dislin_Scope()
	{}

	void Dislin_Scope::initialize(char scope_type, char scope_level, int scope_size, int plot_size)
	{
		nb_data_scope = scope_size;
		nb_data_to_plot = plot_size;
		nb_plotted = 0;
		var_plotted = 0;

		switch (scope_type)
		{
		case EMG_SCOPE:
			//create EMG scope
			diag_type = EMG_SCOPE;
			if (emg_precision == EMG_16BIT_PRECISION)
			{
				//lowy = (int)POS_MIN_CTRL*PRECISION_16BIT_SCALE; highy = (int)POS_MAX_CTRL*PRECISION_16BIT_SCALE;
				//stepy = (int) STEP_CTRL*PRECISION_16BIT_SCALE;
				lowy = -30; highy = 30;
				stepy = 10;
			}
			else if (emg_precision = EMG_24BIT_PRECISION)
			{
				lowy = -150; highy = 150;
				stepy = 50;
			}
			//lowy = -300; highy = 300; //2 EMG
			
			diag.metafl("xwi1");
			diag.window(800, (scope_level - 1) * 280, 400, 250); //Livrable 11
			//diag.window(670, (scope_level - 1)*280, 1060, 250);
			diag.setpag("da0l");
			diag.x11mod("nostore");
			diag.scrmod("revers"); //Livrable 11
			diag.disini();
			diag.axslen(10300, 7500);
			diag.labels("none", "x");
			diag.axspos(1500, 8000);
			diag.height(250);
			diag.errmod("all", "off");
			diag.graf(0.0, scope_size, 0.0, scope_size / 2, lowy, highy, lowy, stepy);

			//init arrays
			emg_window = new float[scope_size];
			xemg_window = new float[scope_size];
			emg_to_plot = new float[plot_size];
			xemg_to_plot = new float[plot_size];
			break;

		case TKE_SCOPE:
			//create TKE scope
			diag_type = TKE_SCOPE;
			lowy = -100; highy = 5e3; stepy = 1000;
			diag.metafl("xwi2");
			diag.window(800, (scope_level - 1) * 280, 400, 250); //Livrable 11
			diag.setpag("da0l");
			diag.x11mod("nostore");
			diag.scrmod("revers"); //Livrable 11
			diag.disini();
			diag.axslen(10300, 7500);
			diag.labels("none", "x");
			diag.axspos(1500, 8000);
			diag.height(250);
			diag.errmod("all", "off");
			diag.graf(0.0, scope_size, 0.0, scope_size / 2, lowy, highy, lowy, stepy);

			//init arrays
			tke_window = new float[scope_size];
			xtke_window = new float[scope_size];
			tke_to_plot = new float[plot_size];
			xtke_to_plot = new float[plot_size];
			break;

		case IMU_SCOPE:
			//create IMU scope
			diag_type = IMU_SCOPE;
			lowy = -80; highy = 80; stepy = 20;
			diag.metafl("xwi3");
			diag.window(800, (scope_level - 1) * 280, 400, 250); //Livrable 11
			diag.setpag("da0l");
			diag.x11mod("nostore");
			diag.scrmod("revers"); //Livrable 11
			diag.disini();
			diag.axslen(10300, 7500);
			diag.labels("none", "x");
			diag.axspos(1500, 8000);
			diag.height(250);
			diag.errmod("all", "off");
			diag.graf(0.0, scope_size, 0.0, scope_size / 2, lowy, highy, lowy, stepy);


			//init arrays
			pitch_window = new float[scope_size];
			roll_window = new float[scope_size];
			yaw_window = new float[scope_size];
			ximu_window = new float[scope_size];
			pitch_to_plot = new float[plot_size];
			roll_to_plot = new float[plot_size];
			yaw_to_plot = new float[plot_size];
			ximu_to_plot = new float[plot_size];
			break;

		case POLAR_SCOPE:
			diag_type = POLAR_SCOPE;
			diag.metafl("xwi1");
			diag.window(800, (scope_level - 1) * 280, 400, 250); //Livrable 11
			diag.setpag("da0l");
			diag.x11mod("nostore");
			diag.scrmod("revers"); //Livrable 11
			diag.disini();
			diag.axslen(10300, 7500);
			diag.labels("none", "x");
			diag.axspos(1500, 8000);
			diag.height(250);
			diag.hsymbl(120);
			diag.errmod("all", "off");
			diag.grafp(1.0, 0.0, 1.0, 0.0, 90.0);

			/*diag.color("white");
			diag.curve(xray, yray, n);*/

			//init arrays
			r = new float[plot_size];
			phi = new float[plot_size];
			r_prev = new float[scope_size];
			phi_prev = new float[scope_size];
			
			const int size_axis = 10;
			r_axis = new float[size_axis];
			phi_x = new float[size_axis];
			phi_y = new float[size_axis];

			r_min = new float[200];
			r_max = new float[200];
			phi_calib = new float[200];
			r_border = new float[200];
			phi_border = new float[400];

			int hx; 
			for (hx = 0; hx <= size_axis-1; hx++)
			{
				if (hx <= (size_axis/2 - 1))
					r_axis[hx] = ((size_axis / 2 - 1) - hx) / (size_axis / 2 - 1);
				else
					r_axis[hx] = (hx - (size_axis / 2)) / (size_axis / 2 - 1);

				phi_x[hx] = hx * M_PI / (size_axis - 1);
				phi_y[hx] = hx * M_PI / (size_axis - 1) + M_PI / 2;
			}
			break;

		defalut:
			break;
		}
	}


	void Dislin_Scope::initialize(char scope_type, char scope_level, int scope_size, int plot_size, int nb_curves)
	{
		initialize(scope_type, scope_level, scope_size, plot_size);
		if (nb_curves != 1)
		{
			nb2_plotted = 0;
			var2_plotted = 0;

			switch (scope_type)
			{
			case EMG_SCOPE:
				emg2_window = new float[scope_size];
				xemg2_window = new float[scope_size];
				emg2_to_plot = new float[plot_size];
				xemg2_to_plot = new float[plot_size];
				break;

			case TKE_SCOPE:
				xTHtke = new float[scope_size];
				THtke_window = new float[scope_size];
				THuptke_window = new float[scope_size];
				THlowtke_window = new float[scope_size];
				for (int pt = 0; pt < nb_data_scope; pt++)
				{
					xTHtke[pt] = pt;
					THtke_window[pt] = 0;
					THuptke_window[pt] = 0;
					THlowtke_window[pt] = 0;
				}
				break;

			case IMU_SCOPE:
				xref_to_plot = new float[plot_size];
				yawref_to_plot = new float[plot_size];
				xref_window = new float[plot_size];
				yawref_window = new float[plot_size];
				break;

			defalut:
				break;
			}
		}
	}

	void Dislin_Scope::update(float *new_data, int size_new)
	{
		switch (diag_type)
		{
		case EMG_SCOPE:
			//update emg
			if (nb_plotted < nb_data_scope && size_new == nb_data_to_plot)// && NB2_PLOTTED < NB_SCOPE_SAMPLES)
			{
				for (int pt = 0; pt < size_new; pt++)
				{
					emg_to_plot[pt + var_plotted] = new_data[pt];
					xemg_to_plot[pt + var_plotted] = nb_plotted + pt;
					emg_window[pt + nb_plotted] = new_data[pt];
					xemg_window[pt + nb_plotted] = nb_plotted + pt;
				}
				var_plotted += size_new;
				nb_plotted += size_new;
				if (var_plotted == nb_data_to_plot)
				{
					diag.color("red");
					diag.curve(xemg_to_plot, emg_to_plot, nb_data_to_plot);
					var_plotted = 0;
				}
			}
			if (nb_plotted == nb_data_scope)
			{
				diag.color("black");
				diag.curve(xemg_window, emg_window, nb_data_scope);
				nb_plotted = 0;
				var_plotted = 0;
			}
			break;

		case TKE_SCOPE:
			//update tke
			if (nb_plotted < nb_data_scope && size_new == nb_data_to_plot)// && NB2_PLOTTED < NB_SCOPE_SAMPLES)
			{
				for (int pt = 0; pt < size_new; pt++)
				{
					tke_to_plot[pt + var_plotted] = new_data[pt];
					xtke_to_plot[pt + var_plotted] = nb_plotted + pt;
					tke_window[pt + nb_plotted] = new_data[pt];
					xtke_window[pt + nb_plotted] = nb_plotted + pt;
				}
				var_plotted += size_new;
				nb_plotted += size_new;
				if (var_plotted == nb_data_to_plot)
				{
					diag.color("red");
					diag.curve(xtke_to_plot, tke_to_plot, nb_data_to_plot);
					var_plotted = 0;
				}
			}
			if (nb_plotted == nb_data_scope)
			{
				diag.color("black");
				diag.curve(xtke_window, tke_window, nb_data_scope);
				nb_plotted = 0;
				var_plotted = 0;
			}
			break;

		case IMU_SCOPE:
			//update imu
			if (nb_plotted < nb_data_scope && size_new == 3)
			{
				ximu_to_plot[var_plotted] = nb_plotted;
				pitch_to_plot[var_plotted] = new_data[0];
				roll_to_plot[var_plotted] = new_data[1];
				yaw_to_plot[var_plotted] = new_data[2];
				ximu_window[nb_plotted] = nb_plotted;
				pitch_window[nb_plotted] = new_data[0];
				roll_window[nb_plotted] = new_data[1];
				yaw_window[nb_plotted] = new_data[2];
				/*yawref_to_plot[var_plotted] = CJACO.yaw_ref[2];
				yawref_window[var_plotted] = CJACO.yaw_ref[2];*/
				var_plotted += 1;
				nb_plotted += 1;
				if (var_plotted == nb_data_to_plot)
				{
					//pitch
					diag.color("red");
					diag.curve(ximu_to_plot, pitch_to_plot, nb_data_to_plot);
					//roll
					diag.color("blue");
					diag.curve(ximu_to_plot, roll_to_plot, nb_data_to_plot);
					//yaw
					diag.color("white");
					diag.curve(ximu_to_plot, yaw_to_plot, nb_data_to_plot);
					var_plotted = 0;
				}
			}
			if (nb_plotted == nb_data_scope)
			{
				diag.color("black");
				//pitch
				diag.curve(ximu_window, pitch_window, nb_data_scope);
				diag.curve(ximu_window, roll_window, nb_data_scope);
				diag.curve(ximu_window, yaw_window, nb_data_scope);
				nb_plotted = 0;
				var_plotted = 0;
			}
			break;
		case POLAR_SCOPE:
			if (nb_plotted < nb_data_scope && size_new == 2)
			{
				r[var_plotted] = new_data[0];
				phi[var_plotted] = new_data[1];

				r_prev[nb_plotted] = new_data[0];
				phi_prev[nb_plotted] = new_data[1];
				
				var_plotted += 1;
				nb_plotted += 1;

				if (var_plotted == 2)
				{
					diag.linwid(100);
					diag.color("red");
					int u;
					//for (u = 0; u < 2; u++)
						//diag.symbol(10, r[u] * cos(phi[u]), r[u] * sin(phi[u]));
					diag.curve(r, phi, 2);
					var_plotted = 0;
				}
			}
			if (nb_plotted == 30)//nb_data_scope)
			{
				diag.linwid(100);
				diag.color("black");
				int u;
				//for (u = 0; u < 4; u++)
					//diag.symbol(10, r_prev[u] * cos(phi_prev[u]), r_prev[u] * sin(phi_prev[u]));
				diag.curve(r_prev, phi_prev, 30);

				diag.linwid(4);
				diag.color("white");
				diag.curve(r_axis, phi_x, 10);
				diag.curve(r_axis, phi_y, 10);

				nb_plotted = 0;
				var_plotted = 0;
			}

			break;
		}
	}

	void Dislin_Scope::update(float *new_data, int size_new, char signal_id)
	{
		if (signal_id == 1)
			update(new_data, size_new);
		else if (signal_id == 2)
		{
			switch (diag_type)
			{
			case EMG_SCOPE:
				//update emg
				if (nb2_plotted < nb_data_scope && size_new == nb_data_to_plot)// && NB2_PLOTTED < NB_SCOPE_SAMPLES)
				{
					for (int pt = 0; pt < size_new; pt++)
					{
						emg2_to_plot[pt + var2_plotted] = new_data[pt];
						xemg2_to_plot[pt + var2_plotted] = nb2_plotted + pt;
						emg2_window[pt + nb2_plotted] = new_data[pt];
						xemg2_window[pt + nb2_plotted] = nb2_plotted + pt;
					}
					var2_plotted += size_new;
					nb2_plotted += size_new;
					if (var2_plotted == nb_data_to_plot)
					{
						diag.color("blue");
						diag.curve(xemg2_to_plot, emg2_to_plot, nb_data_to_plot);
						var2_plotted = 0;
					}
				}
				if (nb2_plotted == nb_data_scope)
				{
					diag.color("black");
					diag.curve(xemg2_window, emg2_window, nb_data_scope);
					nb2_plotted = 0;
					var2_plotted = 0;
				}
				break;

			case TKE_SCOPE:
				//update tke
				diag.color("black");
				diag.curve(xTHtke, THtke_window, nb_data_scope);
				THtke_window[0] = *new_data;
				for (int pt = 1; pt < nb_data_scope; pt++)
				{
					THtke_window[pt] = 0;
					THtke_window[pt] = THtke_window[0];
					xTHtke[pt] = pt;
				}
				diag.color("blue");
				diag.curve(xTHtke, THtke_window, nb_data_scope);
				break;
			}
		}
	}

	void Dislin_Scope::update(float new_data, int size_new, char signal_id)
	{
		if (signal_id == 2)
		{
			switch (diag_type)
			{
			case EMG_SCOPE:
				//
				break;

			case TKE_SCOPE:
				//update tke
				diag.color("black");
				diag.curve(xTHtke, THtke_window, nb_data_scope);
				for (int pt = 0; pt < nb_data_scope; pt++)
				{
					THtke_window[pt] = new_data;
					xTHtke[pt] = pt;
				}
				diag.color("blue");
				diag.curve(xTHtke, THtke_window, nb_data_scope);
				break;
			}
		}
	}

	void Dislin_Scope::update(float threshold_up, float threshold_low, int size_new, char signal_id)
	{
		if (signal_id == 3)
		{
			switch (diag_type)
			{
			case EMG_SCOPE:
				//
				break;

			case TKE_SCOPE:
				//update tke
				diag.color("black");
				diag.curve(xTHtke, THuptke_window, nb_data_scope);
				diag.curve(xTHtke, THlowtke_window, nb_data_scope);
				for (int pt = 0; pt < nb_data_scope; pt++)
				{
					THuptke_window[pt] = threshold_up;
					THlowtke_window[pt] = threshold_low;
					xTHtke[pt] = pt;
				}
				diag.color("yellow");
				diag.curve(xTHtke, THuptke_window, nb_data_scope);
				diag.color("green");
				diag.curve(xTHtke, THlowtke_window, nb_data_scope);
				break;
			}
		}
	}
}