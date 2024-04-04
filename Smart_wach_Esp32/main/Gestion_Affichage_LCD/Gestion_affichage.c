#include "Gestion_affichage.h"

// NB : VERSION LVGL UTILISEE : 7.11


extern lv_chart_series_t *temp_series, *Mag_series_x, *Mag_series_y, *Mag_series_z;
extern lv_chart_series_t *Ang_series_x, *Ang_series_y, *Ang_series_z;
extern lv_chart_series_t *Acc_series_x, *Acc_series_y, *Acc_series_z;
extern lv_style_t style_screen_black;
extern lv_style_t style_label_tab_mesures;
extern lv_style_t style_label_tableau_mesures;
extern lv_style_t style_chart;
extern lv_obj_t *screen[8];
extern lv_obj_t *Graphe_Temperature;
extern lv_obj_t *Graphe_Magnetometrie;
extern lv_obj_t *Graphe_Gyroscopie;
extern lv_obj_t *Graphe_Accelerometrie;
extern lv_obj_t *scr1_label1,
				*scr2_label1,
				*scr3_label1;
extern lv_obj_t *tableau_mesures;

extern QueueHandle_t xQueue_T;
extern QueueHandle_t xQueue_Mag;
extern QueueHandle_t xQueue_Ang;
extern QueueHandle_t xQueue_Acc;
extern TaskHandle_t Affichage_LCD_Handle;
extern uint8_t screen_number;
extern SemaphoreHandle_t screen_number_mutex;
extern lv_obj_t *tableau_mesures;
extern lv_style_t style_time;
extern lv_style_t style_label_graphs;
extern lv_style_t style_chart_graphs;
extern lv_style_t style_custom;



extern struct ValueToSend_Task {
	  float x;
	  float y;
	  float z;
	};

extern lv_style_t style_label_graphs;
extern lv_style_t style_chart_graphs;




void init_screen_1_Affichage_Horodate(void)


{
	screen[0] = lv_obj_create(NULL, NULL);
	lv_style_init(&style_custom);
	lv_style_set_bg_color(&style_custom, LV_STATE_DEFAULT, lv_color_make(45, 96, 71));
	lv_style_set_bg_grad_color(&style_custom, LV_STATE_DEFAULT, lv_color_make(12, 96, 120));
	lv_style_set_bg_grad_dir(&style_custom, LV_STATE_DEFAULT, LV_GRAD_DIR_VER);
	lv_style_set_bg_opa(&style_custom, LV_STATE_DEFAULT, LV_OPA_COVER);
	lv_obj_add_style(screen[0], LV_OBJ_PART_MAIN, &style_custom);
	scr1_label1 =  lv_label_create(screen[0], NULL);
	lv_obj_set_width(scr1_label1, LV_HOR_RES - 20);
	lv_obj_set_width(scr1_label1, LV_HOR_RES);
	lv_label_set_align(scr1_label1, LV_LABEL_ALIGN_CENTER); //
	lv_obj_align(scr1_label1, NULL, LV_ALIGN_CENTER, 0, -20);
	lv_style_set_text_color(&style_time, LV_STATE_DEFAULT, LV_COLOR_WHITE);
	lv_style_set_text_font(&style_time, LV_STATE_DEFAULT, &lv_font_montserrat_24);
	lv_obj_add_style(scr1_label1, LV_LABEL_PART_MAIN, &style_time);
}


void init_screen_3_graphe_temperature(void)
{
		lv_style_init(&style_chart_graphs);
		lv_style_set_text_color(&style_label_graphs, LV_STATE_DEFAULT, LV_COLOR_WHITE);
		lv_style_init(&style_screen_black);
		lv_style_set_pad_bottom(&style_chart_graphs, LV_STATE_DEFAULT, 40);
		lv_style_set_pad_left(&style_chart_graphs, LV_STATE_DEFAULT, 70);
		lv_style_set_pad_right(&style_chart_graphs, LV_STATE_DEFAULT, 20);
		lv_style_set_pad_top(&style_chart_graphs, LV_STATE_DEFAULT, 15);
		lv_style_set_bg_color(&style_screen_black, LV_STATE_DEFAULT, LV_COLOR_BLACK);
		lv_style_set_text_font(&style_chart_graphs, LV_STATE_DEFAULT, &lv_font_montserrat_14);
		screen[2] = lv_obj_create(NULL, NULL);
		lv_obj_add_style(screen[2], LV_OBJ_PART_MAIN, &style_screen_black);
		Graphe_Temperature = lv_chart_create(screen[2], NULL);
		lv_chart_set_point_count(Graphe_Temperature, 10);
		lv_chart_set_y_tick_texts(Graphe_Temperature, "27°C\n24°C\n21°C\n18°C\n15°C\n12°C\n9°C\n6°C\n3°C\n0°C", 1, LV_CHART_AXIS_SKIP_LAST_TICK);
		lv_chart_set_x_tick_texts(Graphe_Temperature, "-9s\n-8s\n-7s\n-6s\n-5s\n-4s\n-3s\n-2s\n-1s\nNow", 2, LV_CHART_AXIS_SKIP_LAST_TICK);
		lv_obj_set_size(Graphe_Temperature, 320, 210);
		lv_chart_set_range(Graphe_Temperature, 0, 30000);
		lv_obj_align(Graphe_Temperature, NULL, LV_ALIGN_CENTER, 0, 10);
		lv_chart_set_type(Graphe_Temperature, LV_CHART_TYPE_LINE);
		lv_obj_t * Graphe_Temperature_title = lv_label_create(screen[2], NULL);
		lv_label_set_text(Graphe_Temperature_title, "Temperature en temps reel");
		lv_obj_add_style(Graphe_Temperature, LV_CHART_PART_BG, &style_chart_graphs);
		lv_obj_add_style(Graphe_Temperature, LV_CHART_PART_SERIES, &style_chart_graphs);
		lv_obj_add_style(Graphe_Temperature_title, LV_LABEL_PART_MAIN, &style_label_graphs);
		lv_obj_align(Graphe_Temperature_title, Graphe_Temperature, LV_ALIGN_OUT_TOP_MID, 0, 0);
		temp_series = lv_chart_add_series(Graphe_Temperature, LV_COLOR_RED);
		}

void init_screen_4_graphe_magnetometre(void)

{
		lv_style_init(&style_chart_graphs);
		lv_style_set_text_color(&style_label_graphs, LV_STATE_DEFAULT, LV_COLOR_WHITE);
		lv_style_init(&style_screen_black);
		lv_style_set_pad_bottom(&style_chart_graphs, LV_STATE_DEFAULT, 40);
		lv_style_set_pad_left(&style_chart_graphs, LV_STATE_DEFAULT, 70);
		lv_style_set_pad_right(&style_chart_graphs, LV_STATE_DEFAULT, 20);
		lv_style_set_pad_top(&style_chart_graphs, LV_STATE_DEFAULT, 15);
		lv_style_set_bg_color(&style_screen_black, LV_STATE_DEFAULT, LV_COLOR_BLACK);
		lv_style_set_text_font(&style_chart_graphs, LV_STATE_DEFAULT, &lv_font_montserrat_14);
		screen[3] = lv_obj_create(NULL, NULL);
		lv_obj_add_style(screen[3], LV_OBJ_PART_MAIN, &style_screen_black);
		Graphe_Magnetometrie = lv_chart_create(screen[3], NULL);
		lv_chart_set_point_count(Graphe_Magnetometrie, 11);
		lv_chart_set_y_tick_texts(Graphe_Magnetometrie, "600\n480\n360\n240\n120\n0\n-120\n-240\n-360\n-480\n-600", 1, LV_CHART_AXIS_SKIP_LAST_TICK);
		lv_chart_set_x_tick_texts(Graphe_Magnetometrie, "-9s\n-8s\n-7s\n-6s\n-5s\n-4s\n-3s\n-2s\n-1s\n0s", 2, LV_CHART_AXIS_SKIP_LAST_TICK);
		lv_obj_set_size(Graphe_Magnetometrie, 320, 210);
		lv_chart_set_range(Graphe_Magnetometrie, -600, 600);
		lv_obj_align(Graphe_Magnetometrie, NULL, LV_ALIGN_CENTER, 0, 10);
		lv_chart_set_type(Graphe_Magnetometrie, LV_CHART_TYPE_LINE);
		lv_obj_t * Magnetometre_Temperature_title = lv_label_create(screen[3], NULL);
		lv_label_set_text(Magnetometre_Temperature_title, "Champs Magnetique(mG) - Temps Reel");
		lv_obj_add_style(Graphe_Magnetometrie, LV_CHART_PART_BG, &style_chart_graphs);
		lv_obj_add_style(Graphe_Magnetometrie, LV_CHART_PART_SERIES, &style_chart_graphs);
		lv_obj_add_style(Magnetometre_Temperature_title, LV_LABEL_PART_MAIN, &style_label_graphs);
		lv_obj_align(Magnetometre_Temperature_title, Graphe_Magnetometrie, LV_ALIGN_OUT_TOP_MID, 0, 0);
		Mag_series_x = lv_chart_add_series(Graphe_Magnetometrie, LV_COLOR_GREEN);
		Mag_series_y = lv_chart_add_series(Graphe_Magnetometrie, LV_COLOR_BLUE);
		Mag_series_z = lv_chart_add_series(Graphe_Magnetometrie, LV_COLOR_PURPLE);
}


void init_screen_5_graphe_gyroscope(void)

{
		lv_style_init(&style_chart_graphs);
		lv_style_set_text_color(&style_label_graphs, LV_STATE_DEFAULT, LV_COLOR_WHITE);
		lv_style_init(&style_screen_black);
		lv_style_set_pad_bottom(&style_chart_graphs, LV_STATE_DEFAULT, 40);
		lv_style_set_pad_left(&style_chart_graphs, LV_STATE_DEFAULT, 70);
		lv_style_set_pad_right(&style_chart_graphs, LV_STATE_DEFAULT, 20);
		lv_style_set_pad_top(&style_chart_graphs, LV_STATE_DEFAULT, 15);
		lv_style_set_bg_color(&style_screen_black, LV_STATE_DEFAULT, LV_COLOR_BLACK);
		lv_style_set_text_font(&style_chart_graphs, LV_STATE_DEFAULT, &lv_font_montserrat_14);
		screen[4] = lv_obj_create(NULL, NULL);
		lv_obj_add_style(screen[4], LV_OBJ_PART_MAIN, &style_screen_black);
		Graphe_Gyroscopie = lv_chart_create(screen[4], NULL);
		lv_chart_set_point_count(Graphe_Gyroscopie, 11);
		lv_chart_set_y_tick_texts(Graphe_Gyroscopie, "1000\n800\n600\n400\n200\n0\n-200\n-400\n-600\n-800\n-1000", 1, LV_CHART_AXIS_SKIP_LAST_TICK);
		lv_chart_set_x_tick_texts(Graphe_Gyroscopie, "-9s\n-8s\n-7s\n-6s\n-5s\n-4s\n-3s\n-2s\n-1s\n0s", 2, LV_CHART_AXIS_SKIP_LAST_TICK);
		lv_obj_set_size(Graphe_Gyroscopie, 320, 210);
		lv_chart_set_range(Graphe_Gyroscopie, -1000, 1000);
		lv_obj_align(Graphe_Gyroscopie, NULL, LV_ALIGN_CENTER, 0, 10);
		lv_chart_set_type(Graphe_Gyroscopie, LV_CHART_TYPE_LINE);
		lv_obj_t * Gyroscopie_title = lv_label_create(screen[4], NULL);
		lv_label_set_text(Gyroscopie_title, "Vitesse angulaire(mdps) - Temps Reel");
		lv_obj_add_style(Graphe_Gyroscopie, LV_CHART_PART_BG, &style_chart_graphs);
		lv_obj_add_style(Graphe_Gyroscopie, LV_CHART_PART_SERIES, &style_chart_graphs);
		lv_obj_add_style(Gyroscopie_title, LV_LABEL_PART_MAIN, &style_label_graphs);
		lv_obj_align(Gyroscopie_title, Graphe_Gyroscopie, LV_ALIGN_OUT_TOP_MID, 0, 0);
		Ang_series_x = lv_chart_add_series(Graphe_Gyroscopie, LV_COLOR_GREEN);
		Ang_series_y = lv_chart_add_series(Graphe_Gyroscopie, LV_COLOR_BLUE);
		Ang_series_z = lv_chart_add_series(Graphe_Gyroscopie, LV_COLOR_PURPLE);
}


void init_screen_6_graphe_accelerometre(void)

{
		lv_style_init(&style_chart_graphs);
		lv_style_set_text_color(&style_label_graphs, LV_STATE_DEFAULT, LV_COLOR_YELLOW);
		lv_style_init(&style_screen_black);
		lv_style_set_pad_bottom(&style_chart_graphs, LV_STATE_DEFAULT, 40);
		lv_style_set_pad_left(&style_chart_graphs, LV_STATE_DEFAULT, 70);
		lv_style_set_pad_right(&style_chart_graphs, LV_STATE_DEFAULT, 20);
		lv_style_set_pad_top(&style_chart_graphs, LV_STATE_DEFAULT, 15);
		lv_style_set_bg_color(&style_screen_black, LV_STATE_DEFAULT, LV_COLOR_BLUE);
		lv_style_set_text_font(&style_chart_graphs, LV_STATE_DEFAULT, &lv_font_montserrat_14);
		screen[5] = lv_obj_create(NULL, NULL);
		lv_obj_add_style(screen[5], LV_OBJ_PART_MAIN, &style_screen_black);
		Graphe_Accelerometrie = lv_chart_create(screen[5], NULL);
		lv_chart_set_point_count(Graphe_Accelerometrie, 11);
		lv_chart_set_y_tick_texts(Graphe_Accelerometrie, "1100\n900\n700\n500\n300\n100\n-100\n-300\n-500\n-700\n-900", 1, LV_CHART_AXIS_SKIP_LAST_TICK);
		lv_chart_set_x_tick_texts(Graphe_Accelerometrie, "-9s\n-8s\n-7s\n-6s\n-5s\n-4s\n-3s\n-2s\n-1s\n0s", 2, LV_CHART_AXIS_SKIP_LAST_TICK);
		lv_obj_set_size(Graphe_Accelerometrie, 320, 210);
		lv_chart_set_range(Graphe_Accelerometrie, -900, 1100);
		lv_obj_align(Graphe_Accelerometrie, NULL, LV_ALIGN_CENTER, 0, 10);
		lv_chart_set_type(Graphe_Accelerometrie, LV_CHART_TYPE_LINE);
		lv_obj_t * Accelerometrie_title = lv_label_create(screen[5], NULL);
		lv_label_set_text(Accelerometrie_title, "Acceleration(mg) - Temps Reel");
		lv_obj_add_style(Graphe_Accelerometrie, LV_CHART_PART_BG, &style_chart_graphs);
		lv_obj_add_style(Graphe_Accelerometrie, LV_CHART_PART_SERIES, &style_chart_graphs);
		lv_obj_add_style(Accelerometrie_title, LV_LABEL_PART_MAIN, &style_label_graphs);
		lv_obj_align(Accelerometrie_title, Graphe_Accelerometrie, LV_ALIGN_OUT_TOP_MID, 0, 0);
		Acc_series_x = lv_chart_add_series(Graphe_Accelerometrie, LV_COLOR_GREEN);
		Acc_series_y = lv_chart_add_series(Graphe_Accelerometrie, LV_COLOR_BLUE);
		Acc_series_z = lv_chart_add_series(Graphe_Accelerometrie, LV_COLOR_PURPLE);
}

void init_screen_2_tableau_mesures(void) {

	static lv_style_t style_table_bg;
	lv_style_init(&style_table_bg);
	lv_style_set_bg_color(&style_table_bg, LV_STATE_DEFAULT, LV_COLOR_WHITE);
	lv_style_set_bg_opa(&style_table_bg, LV_STATE_DEFAULT, LV_OPA_COVER);

	static lv_style_t style_bg_cyan;
	lv_style_init(&style_bg_cyan);
	lv_style_set_bg_color(&style_bg_cyan, LV_STATE_DEFAULT, LV_COLOR_GREEN);
	lv_style_set_bg_opa(&style_bg_cyan, LV_STATE_DEFAULT, LV_OPA_COVER);




	static lv_style_t style_header;
	lv_style_init(&style_header);
	lv_style_set_text_color(&style_header, LV_STATE_DEFAULT, LV_COLOR_BLUE);
	lv_style_set_text_font(&style_header, LV_STATE_DEFAULT, &lv_font_montserrat_12);
    lv_style_init(&style_label_tableau_mesures);
    lv_style_set_text_color(&style_label_tab_mesures, LV_STATE_DEFAULT, LV_COLOR_BLACK);
    lv_style_set_text_font(&style_label_tableau_mesures, LV_STATE_DEFAULT, &lv_font_montserrat_12);
    lv_style_set_pad_inner(&style_label_tableau_mesures, LV_STATE_DEFAULT, 4);
    lv_style_set_pad_top(&style_label_tableau_mesures, LV_STATE_DEFAULT, 2);
    lv_style_set_pad_bottom(&style_label_tableau_mesures, LV_STATE_DEFAULT, 2);


    screen[1] = lv_obj_create(NULL, NULL);
    lv_obj_add_style(screen[1], LV_OBJ_PART_MAIN, &style_screen_black);


    lv_obj_t *cont = lv_cont_create(screen[1], NULL);
	lv_obj_set_size(cont, LV_HOR_RES, LV_VER_RES);
	lv_obj_add_style(cont, LV_CONT_PART_MAIN, &style_bg_cyan);
	lv_obj_align(cont, NULL, LV_ALIGN_CENTER, 0, 0);


    static lv_style_t style_transparent;
    lv_style_init(&style_transparent);
    lv_style_set_bg_opa(&style_transparent, LV_STATE_DEFAULT, LV_OPA_TRANSP);

    tableau_mesures = lv_table_create(cont, NULL);
    lv_obj_add_style(tableau_mesures, LV_TABLE_PART_BG, &style_table_bg);


    lv_table_set_col_cnt(tableau_mesures, 2);
    lv_table_set_row_cnt(tableau_mesures, 10);

    lv_obj_add_style(tableau_mesures, LV_TABLE_PART_CELL1, &style_label_tableau_mesures);

    lv_table_set_cell_value(tableau_mesures, 0, 0, "Mesure");
    lv_table_set_cell_value(tableau_mesures, 0, 1, "Valeur");

    lv_obj_align(tableau_mesures, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_table_set_cell_value(tableau_mesures, 1, 0, "Temperature");
    lv_table_set_cell_value(tableau_mesures, 2, 0, "Accel en X");
    lv_table_set_cell_value(tableau_mesures, 3, 0, "Accel en Y");
    lv_table_set_cell_value(tableau_mesures, 4, 0, "Accel en Z");
    lv_table_set_cell_value(tableau_mesures, 5, 0, "Magne en X");
    lv_table_set_cell_value(tableau_mesures, 6, 0, "Magne en Y");
    lv_table_set_cell_value(tableau_mesures, 7, 0, "Magne en Z");
    lv_table_set_cell_value(tableau_mesures, 8, 0, "Gyro en X");
    lv_table_set_cell_value(tableau_mesures, 9, 0, "Gyro en Y");
    lv_table_set_cell_value(tableau_mesures, 10, 0, "Gyro en Z");


    lv_table_set_cell_value(tableau_mesures, 1, 1, "0°C");
    lv_table_set_cell_value(tableau_mesures, 2, 1, "0 mg");
    lv_table_set_cell_value(tableau_mesures, 3, 1, "0 mg");
    lv_table_set_cell_value(tableau_mesures, 4, 1, "0 mg");
    lv_table_set_cell_value(tableau_mesures, 5, 1, "0 mG");
    lv_table_set_cell_value(tableau_mesures, 6, 1, "0 mG");
    lv_table_set_cell_value(tableau_mesures, 7, 1, "0 mG");
    lv_table_set_cell_value(tableau_mesures, 8, 1, "0 rad/s");
    lv_table_set_cell_value(tableau_mesures, 9, 1, "0 rad/s");
    lv_table_set_cell_value(tableau_mesures, 10, 1, "0 rad/s");
    lv_obj_add_style(tableau_mesures, LV_TABLE_PART_CELL1, &style_header);
    for (int row = 1; row < lv_table_get_row_cnt(tableau_mesures); row++) {
        for (int col = 0; col < lv_table_get_col_cnt(tableau_mesures); col++) {
            lv_table_set_cell_type(tableau_mesures, row, col, 2);
        }
    }
    lv_obj_add_style(tableau_mesures, LV_TABLE_PART_CELL2, &style_label_tableau_mesures);
    lv_table_set_cell_type(tableau_mesures, 0, 0, 1);
    lv_table_set_cell_type(tableau_mesures, 0, 1, 1);

}




void Affichage_LCD(void *args) /* DISPLAY MANAGEMENT TASK */
{
	float receivedTemperature;
	struct ValueToSend_Task receivedValue_Magneto, receivedValue_Gyro, receivedValue_Accelero;
	char datetime_buf[250];
	int temp_for_chart;
	struct tm timeinfo_temps_reel;

	while(1)
	{
		if(screen_number == 2)

		{
			if(xQueueReceive(xQueue_T, &receivedTemperature, portMAX_DELAY) == pdPASS)
					{

					            temp_for_chart = (int)(receivedTemperature * 1000);
					            lv_chart_set_next(Graphe_Temperature, temp_series, temp_for_chart);
					}
		}

		else if(screen_number == 3)
		{
			 if (xQueueReceive(xQueue_Mag, &receivedValue_Magneto, portMAX_DELAY) == pdPASS)
			{

				lv_chart_set_next(Graphe_Magnetometrie, Mag_series_x, receivedValue_Magneto.x);
				lv_chart_set_next(Graphe_Magnetometrie, Mag_series_y, receivedValue_Magneto.y);
				lv_chart_set_next(Graphe_Magnetometrie, Mag_series_z, receivedValue_Magneto.z);
			}
		}

		else if(screen_number == 4)
				{
					 if (xQueueReceive(xQueue_Ang, &receivedValue_Gyro, portMAX_DELAY) == pdPASS)
					{

						lv_chart_set_next(Graphe_Gyroscopie, Ang_series_x, (receivedValue_Gyro.x * 0.001745));
						lv_chart_set_next(Graphe_Gyroscopie, Ang_series_y, (receivedValue_Gyro.y * 0.001745));
						lv_chart_set_next(Graphe_Gyroscopie, Ang_series_z, (receivedValue_Gyro.z * 0.001745));
					}
				}
		else if(screen_number == 5)
						{
							 if (xQueueReceive(xQueue_Acc, &receivedValue_Accelero, portMAX_DELAY) == pdPASS)
							{

								lv_chart_set_next(Graphe_Accelerometrie, Acc_series_x, receivedValue_Accelero.x);
								lv_chart_set_next(Graphe_Accelerometrie, Acc_series_y, receivedValue_Accelero.y);
								lv_chart_set_next(Graphe_Accelerometrie, Acc_series_z, receivedValue_Accelero.z);
							}
						}
		else if(screen_number == 1) {

			if(xQueueReceive(xQueue_T, &receivedTemperature, portMAX_DELAY) == pdPASS) {
			                char tempVal[16];
			                snprintf(tempVal, sizeof(tempVal), "%.2f°C", receivedTemperature);
			                lv_table_set_cell_value(tableau_mesures, 1, 1, tempVal);
			            }

			if(xQueueReceive(xQueue_Acc, &receivedValue_Accelero, portMAX_DELAY) == pdPASS) {
			                char accXVal[16];
			                char accYVal[16];
			                char accZVal[16];
			                snprintf(accXVal, sizeof(accXVal), "%.2f mg", receivedValue_Accelero.x);
			                snprintf(accYVal, sizeof(accZVal), "%.2f mg", receivedValue_Accelero.y);
			                snprintf(accZVal, sizeof(accZVal), "%.2f mg", receivedValue_Accelero.z);
			                lv_table_set_cell_value(tableau_mesures, 2, 1, accXVal);
			                lv_table_set_cell_value(tableau_mesures, 3, 1, accYVal);
			                lv_table_set_cell_value(tableau_mesures, 4, 1, accZVal);
			            }
			if(xQueueReceive(xQueue_Mag, &receivedValue_Magneto, portMAX_DELAY) == pdPASS) {
						    char magXVal[16];
						    char magYVal[16];
						    char magZVal[16];
						    snprintf(magXVal, sizeof(magXVal), "%.2f mG", receivedValue_Magneto.x);
						    snprintf(magYVal, sizeof(magYVal), "%.2f mG", receivedValue_Magneto.y);
						    snprintf(magZVal, sizeof(magZVal), "%.2f mG", receivedValue_Magneto.z);
						    lv_table_set_cell_value(tableau_mesures, 5, 1, magXVal);
						    lv_table_set_cell_value(tableau_mesures, 6, 1, magYVal);
						    lv_table_set_cell_value(tableau_mesures, 7, 1, magZVal);
						}
			if(xQueueReceive(xQueue_Ang, &receivedValue_Gyro, portMAX_DELAY) == pdPASS) {
						    char angXVal[16];
						    char angYVal[16];
							char angZVal[16];
							snprintf(angXVal, sizeof(angXVal), "%.2f rad/s", (receivedValue_Gyro.x * 0.001745));
							snprintf(angYVal, sizeof(angYVal), "%.2f rad/s", (receivedValue_Gyro.y * 0.001745));
							snprintf(angZVal, sizeof(angZVal), "%.2f rad/s", (receivedValue_Gyro.z * 0.001745));
							lv_table_set_cell_value(tableau_mesures, 8, 1, angXVal);
							lv_table_set_cell_value(tableau_mesures, 9, 1, angYVal);
							lv_table_set_cell_value(tableau_mesures, 10, 1, angZVal);
						}
		}


		timeinfo_temps_reel = Lecture_RTC();
		strftime(datetime_buf, sizeof(datetime_buf), "\nMontre Connectee \n***Ecran d'Accueil***\nTemps Reel : %d/%m/%Y\n%H:%M:%S \n", &timeinfo_temps_reel);
		lv_label_set_text(scr1_label1, datetime_buf);
		lv_obj_align(scr1_label1, NULL, LV_ALIGN_CENTER, 0, 0);

		if(xSemaphoreTake(screen_number_mutex, portMAX_DELAY) == pdTRUE) {
		    lv_scr_load(screen[screen_number]);
		    xSemaphoreGive(screen_number_mutex);
		}
		lv_task_handler();
		vTaskSuspend(Affichage_LCD_Handle);
	}
}




