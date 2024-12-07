#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_vfs.h"
#include "esp_spiffs.h"

#include "lcd_com.h"
#include "lcd_lib.h"
#include "fontx.h"
#include "bmpfile.h"
#include "decode_jpeg.h"
#include "decode_png.h"
#include "pngle.h"
#include "fonctions.c"



void TFT(void *pvParameters)
{
	// IMPORTER LES FONTS


	FontxFile fx16G[2];
	FontxFile fx24G[2];
	FontxFile fx32G[2];
	InitFontx(fx16G,"/spiffs/ILGH16XB.FNT",""); // 8x16Dot Gothic
	InitFontx(fx24G,"/spiffs/ILGH24XB.FNT",""); // 12x24Dot Gothic
	InitFontx(fx32G,"/spiffs/ILGH32XB.FNT",""); // 16x32Dot Gothic

	FontxFile fx32L[2];
	InitFontx(fx32L,"/spiffs/LATIN32B.FNT",""); // 16x32Dot Latinc

	FontxFile fx16M[2];
	FontxFile fx24M[2];
	FontxFile fx32M[2];
	InitFontx(fx16M,"/spiffs/ILMH16XB.FNT",""); // 8x16Dot Mincyo
	InitFontx(fx24M,"/spiffs/ILMH24XB.FNT",""); // 12x24Dot Mincyo
	InitFontx(fx32M,"/spiffs/ILMH32XB.FNT",""); // 16x32Dot Mincyo
	// IMPORTATION FINIE
	
	//INITIALISER LE PROTOCOL DE COMMUNICATION DE L'ECRAN GRAPHIQUE, DANS NOTRE CAS, COMMUNICATION A REGISTRE
	TFT_t dev;
	lcd_interface_cfg(&dev, INTERFACE);

	//PARAMETRES DE L'ECRAN HAUTER/LARGEUR
	INIT_FUNCTION(&dev, CONFIG_WIDTH, CONFIG_HEIGHT, CONFIG_OFFSETX, CONFIG_OFFSETY);


// INVERSION DES COULEURS OUi/NON

#if CONFIG_INVERSION
	ESP_LOGI(TAG, "Enable Display Inversion");
	lcdInversionOn(&dev);
#endif



/*
Signal analogique pour le tactile * L'ECRAN NE MARCHE PAS *
X+ X- Y+ Y- SEULEMENT X+ ET X- MARCHENT
*/

#if CONFIG_ENABLE_TOUCH

#if CONFIG_XP_GPIO_D6
  int gpio_xp = dev._d0;
#elif CONFIG_XP_GPIO_D0
  int gpio_xp = dev._d0;
#elif CONFIG_XP_GPIO_D1
  int gpio_xp = dev._d1;
#endif

#if CONFIG_XM_GPIO_RS
  int gpio_xm = dev._rs;
#elif CONFIG_XM_GPIO_CS
  int gpio_xm = dev._cs;
#endif

#if CONFIG_YP_GPIO_WR
  int gpio_yp = dev._cs;
#elif CONFIG_YP_GPIO_CS
  int gpio_yp = dev._cs;
#elif CONFIG_YP_GPIO_RS
  int gpio_yp = dev._rs;
#endif

#if CONFIG_YM_GPIO_D7
  int gpio_ym = dev._d1;
#elif CONFIG_YM_GPIO_D0
  int gpio_ym = dev._d0;
#elif CONFIG_YM_GPIO_D1
  int gpio_ym = dev._d1;
#endif
	touch_interface_cfg(&dev, CONFIG_ADC_CHANNEL_YP, CONFIG_ADC_CHANNEL_XM, gpio_xp, gpio_xm, gpio_yp, gpio_ym);
#endif



	while(1) {
		traceHeap();

#if  0 //CONFIG_ENABLE_TOUCH //seulement une partie marche, a ignorer

		TouchCalibration(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT);
		TouchPenTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);

		TouchKeyTest(&dev, fx32G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
		TouchMenuTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
		TouchMoveTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
#ifndef CONFIG_IDF_TARGET_ESP32S2
	TouchIconTest(&dev, fx24G, CONFIG_WIDTH, CONFIG_HEIGHT, 1000);
#endif
#endif
//**********************************************************************************Fonction principale du code commence a partir d'ici *********************************************************
		WAIT;
		char file[32];
		strcpy(file, "/images/jugo.png"); //IMPORTER UNE IMAGE VERS LA MEMOIRE FLASH
		PNGTest(&dev, file, CONFIG_WIDTH, CONFIG_HEIGHT); //AFFICHER L'IMAGE
		WAIT;
		WAIT;

		// Multi Font Test
		uint16_t color;
		uint8_t ascii[40];
		uint16_t margin = 10;
		lcdFillScreen(&dev, BLACK);
		color = WHITE;
		lcdSetFontDirection(&dev, DIRECTION0);
		uint16_t xpos =0 ;
		uint16_t ypos = 90;
		int xd = 0;
		int yd = 1;
	
			xpos = xpos - (32 * xd) - (margin * xd);; //POSITION X DU TEXTE A AFFICHER 
			ypos = ypos + (24 * yd) + (margin * yd); //POSITION Y DU TEXTE A AFFICHER
			strcpy((char *)ascii, "Bonjour, ceci est un");
			lcdDrawString(&dev, fx32G, xpos, ypos, ascii, color);  //&dev:ECRAN SUR LEQUEL ON AFFICHE, fx32G: FONT, xpos/ypos: POSITION,ascii:TEXTE A AFFICHER, color:COULEUR
			strcpy((char *)ascii, "ecran graphique");
			lcdDrawString(&dev, fx32G, xpos+30, ypos+30, ascii, color); //xpos+30 ypos+30: PREMIERE LIGNE OCCUPE TOUT L'ECRAN DEUXIEME LIGNE EST EN DESSOUS (+30)
		
		lcdSetFontDirection(&dev, DIRECTION0); //DIRECTION
		WAIT;
		WAIT;
	} // end while
}
//MEMOIRE FLASH PAS IMPORTANT A COMPRENDRE
static void listSPIFFS(char * path) {
	DIR* dir = opendir(path);
	assert(dir != NULL);
	while (true) {
		struct dirent*pe = readdir(dir);
		if (!pe) break;
		ESP_LOGI(__FUNCTION__,"d_name=%s d_ino=%d d_type=%x", pe->d_name,pe->d_ino, pe->d_type);
	}
	closedir(dir);
}
//SYSTEME SPIFFS POUR ENREGISTRER LES FICHIERS/ IMAGES. PAS IMPORTANT A COMPRENDRE
esp_err_t mountSPIFFS(char * path, char * label, int max_files) {
	esp_vfs_spiffs_conf_t conf = {
		.base_path = path,
		.partition_label = label,
		.max_files = max_files,
		.format_if_mount_failed =true
	};
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret ==ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret== ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)",esp_err_to_name(ret));
		}
		return ret;
	}


	size_t total = 0, used = 0;
	ret = esp_spiffs_info(conf.partition_label, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG,"Failed to get SPIFFS partition information (%s)",esp_err_to_name(ret));
	} else {
		ESP_LOGI(TAG,"Mount %s to %s success", path, label);
		ESP_LOGI(TAG,"Partition size: total: %d, used: %d", total, used);
	}

	return ret;
}


void app_main()
{
	// INITIALISER NVS
	ESP_LOGI(TAG, "Initialize NVS");
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK( err );



	//INITIALISER SPIFFS OU LES IMAGES SONT ENREGISTREES, 3 ESPACE, /icons ET /images SONT OU LES IMAGES SERONT ENREGISTREES
	ESP_LOGI(TAG, "Initializing SPIFFS");
	esp_err_t ret;
	ret = mountSPIFFS("/spiffs", "storage0", 10);
	if (ret != ESP_OK) return;
	listSPIFFS("/spiffs/");

	ret = mountSPIFFS("/icons", "storage1", 10);
	if (ret != ESP_OK) return;
	listSPIFFS("/icons/");

	ret = mountSPIFFS("/images", "storage2", 14);
	if (ret != ESP_OK) return;
	listSPIFFS("/images/");


	//APPELER LA FONCTION PRINCIPALE, OU L'INITIALISATION DE L'ECRAN ET TOUTE AUTRE FONCTION DONT ON A PARLER

	xTaskCreate(TFT, "TFT", 1024*6, NULL, 2, NULL);

}
