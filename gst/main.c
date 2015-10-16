#include <glib.h>
#include <stdio.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/video/videooverlay.h>
#include <SDL/SDL.h>

#include "hu_uti.h"
#include "hu_aap.h"

typedef struct {
	GMainLoop *loop;
	GstPipeline *pipeline;
	GstAppSrc *src;
	GstElement *sink;
	GstElement *decoder;
	GstElement *convert;
	guint sourceid;
} gst_app_t;

static gst_app_t gst_app;

static gboolean read_data(gst_app_t *app)
{
	GstBuffer *buffer;
	guint8 *ptr;
	GstFlowReturn ret;
	int iret;
	char *vbuf;
	int res_len = 0;

	iret = hu_aap_recv_process ();                                    // Process 1 message
	if (iret != 0) {
		printf("hu_aap_recv_process() iret: %d\n", iret);
		return FALSE;
	}

	/* Is there a video buffer queued? */
	vbuf = read_head_buffer_get (&res_len);
	if (vbuf != NULL) {
		ptr = (guint8 *)g_malloc(res_len);
		g_assert(ptr);
		memcpy(ptr, vbuf, res_len);

		buffer = gst_buffer_new_wrapped(ptr, res_len);

		ret = gst_app_src_push_buffer(app->src, buffer);

		if(ret !=  GST_FLOW_OK){
			g_debug("push buffer returned %d for %d bytes \n", ret, res_len);
			return FALSE;
		}
	}

	return TRUE;
}

static void start_feed (GstElement * pipeline, guint size, gst_app_t *app)
{
	if (app->sourceid == 0) {
		GST_DEBUG ("start feeding");
		app->sourceid = g_idle_add ((GSourceFunc) read_data, app);
	}
}

static void stop_feed (GstElement * pipeline, gst_app_t *app)
{
	if (app->sourceid != 0) {
		GST_DEBUG ("stop feeding");
		g_source_remove (app->sourceid);
		app->sourceid = 0;
	}
}

static void on_pad_added(GstElement *element, GstPad *pad)
{
	GstCaps *caps;
	GstStructure *str;
	gchar *name;
	GstPad *convertsink;
	GstPadLinkReturn ret;

	g_debug("pad added");

	caps = gst_pad_get_current_caps(pad);
	str = gst_caps_get_structure(caps, 0);

	g_assert(str);

	name = (gchar*)gst_structure_get_name(str);

	g_debug("pad name %s", name);

	if(g_strrstr(name, "video")){

		convertsink = gst_element_get_static_pad(gst_app.convert, "sink");
		g_assert(convertsink);
		ret = gst_pad_link(pad, convertsink);
		g_debug("pad_link returned %d\n", ret);
		gst_object_unref(convertsink);
	}
	gst_caps_unref(caps);
}

static gboolean bus_callback(GstBus *bus, GstMessage *message, gpointer *ptr)
{
	gst_app_t *app = (gst_app_t*)ptr;

	switch(GST_MESSAGE_TYPE(message)){

		case GST_MESSAGE_ERROR:{
					       gchar *debug;
					       GError *err;

					       gst_message_parse_error(message, &err, &debug);
					       g_print("Error %s\n", err->message);
					       g_error_free(err);
					       g_free(debug);
					       g_main_loop_quit(app->loop);
				       }
				       break;

		case GST_MESSAGE_WARNING:{
						 gchar *debug;
						 GError *err;
						 gchar *name;

						 gst_message_parse_warning(message, &err, &debug);
						 g_print("Warning %s\nDebug %s\n", err->message, debug);

						 name = (gchar *)GST_MESSAGE_SRC_NAME(message);

						 g_print("Name of src %s\n", name ? name : "nil");
						 g_error_free(err);
						 g_free(debug);
					 }
					 break;

		case GST_MESSAGE_EOS:
					 g_print("End of stream\n");
					 g_main_loop_quit(app->loop);
					 break;

		case GST_MESSAGE_STATE_CHANGED:
					 break;

		default:
//					 g_print("got message %s\n", \
							 gst_message_type_get_name (GST_MESSAGE_TYPE (message)));
					 break;
	}

	return TRUE;
}

static int gst_pipeline_init(gst_app_t *app)
{
	GstBus *bus;
	GstStateChangeReturn state_ret;

	gst_init(NULL, NULL);

	app->pipeline = (GstPipeline*)gst_pipeline_new("mypipeline");
	bus = gst_pipeline_get_bus(app->pipeline);
	gst_bus_add_watch(bus, (GstBusFunc)bus_callback, app);
	gst_object_unref(bus);

	app->src = (GstAppSrc*)gst_element_factory_make("appsrc", "mysrc");
	app->decoder = gst_element_factory_make("decodebin", "mydecoder");
	app->convert = gst_element_factory_make("imxipuvideotransform", "myconvert");
	app->sink = gst_element_factory_make("imxipuvideosink", "myvsink");

	g_assert(app->src);
	g_assert(app->decoder);
	g_assert(app->convert);
	g_assert(app->sink);

	g_signal_connect(app->src, "need-data", G_CALLBACK(start_feed), app);
	g_signal_connect(app->src, "enough-data", G_CALLBACK(stop_feed), app);
	g_signal_connect(app->decoder, "pad-added",
			G_CALLBACK(on_pad_added), app->decoder);

	gst_bin_add_many(GST_BIN(app->pipeline), (GstElement*)app->src,
			app->decoder, app->convert, app->sink, NULL);

	if(!gst_element_link((GstElement*)app->src, app->decoder)){
		g_warning("failed to link src anbd decoder");
	}

	if(!gst_element_link(app->convert, app->sink)){
		g_warning("failed to link convert and sink");
	}

	gst_app_src_set_stream_type(app->src, GST_APP_STREAM_TYPE_STREAM);

	return 0;
}

static int aa_cmd_send(int cmd_len, unsigned char *cmd_buf, int res_max, unsigned char *res_buf)
{
	int chan = cmd_buf[0];
	int res_len = 0;
	int ret = 0;
	char *dq_buf;

	res_buf = (unsigned char *)malloc(res_max);
	if (!res_buf) {
		printf("TOTAL FAIL\n");
		return -1;
	}

//	printf("chan: %d cmd_len: %d\n", chan, cmd_len);
	ret = hu_aap_enc_send (chan, cmd_buf+4, cmd_len - 4);
	if (ret < 0) {
		printf("aa_cmd_send(): hu_aap_enc_send() failed with (%d)\n", ret);
		free(res_buf);
		return ret;
	}

	dq_buf = read_head_buffer_get(&res_len);
	if (!dq_buf || res_len <= 0) {
		printf("No data dq_buf!\n");
		free(res_buf);
		return 0;
	}
	memcpy(res_buf, dq_buf, res_len);
	/* FIXME - we do nothing with this crap, probably check for ack and move along */

	free(res_buf);

        return res_len;
}

static size_t uleb128_encode(uint64_t value, uint8_t *data)
{
	uint8_t cbyte;
	size_t enc_size = 0;

	do {
		cbyte = value & 0x7f;
		value >>= 7;
		if (value != 0)
			cbyte |= 0x80;
		data[enc_size++] = cbyte;
	} while (value != 0);

	return enc_size;
}

#define ACTION_DOWN	0
#define ACTION_UP	1
#define ACTION_MOVE	2
#define TS_MAX_REQ_SZ	32
static const uint8_t ts_header[] ={0x02, 0x0b, 0x03, 0x00, 0x80, 0x01, 0x08};
static const uint8_t ts_sizes[] = {0x1a, 0x09, 0x0a, 0x03};
static const uint8_t ts_footer[] = {0x10, 0x00, 0x18};

static void aa_touch_event(uint8_t action, int x, int y) {
	struct timespec tp;
	uint8_t *buf;
	int idx;
	int siz_arr = 0;
	int size1_idx, size2_idx, i;
	int axis = 0;
	int coordinates[3] = {x, y, 0};

	buf = (uint8_t *)malloc(TS_MAX_REQ_SZ);
	if(!buf) {
		printf("Failed to allocate touchscreen event buffer\n");
		return;
	}

	/* Fetch the time stamp */
	clock_gettime(CLOCK_REALTIME, &tp);

	/* Copy header */
	memcpy(buf, ts_header, sizeof(ts_header));
	idx = sizeof(ts_header) +
	      uleb128_encode(tp.tv_nsec, buf + sizeof(ts_header));
	size1_idx = idx + 1;
	size2_idx = idx + 3;

	/* Copy sizes */
	memcpy(buf+idx, ts_sizes, sizeof(ts_sizes));
	idx += sizeof(ts_sizes);

	/* Set magnitude of each axis */
	for (i=0; i<3; i++) {
		axis += 0x08;
		buf[idx++] = axis;
		/* FIXME The following can be optimzed to update size1/2 at end of loop */
		siz_arr = uleb128_encode(coordinates[i], &buf[idx]);
		idx += siz_arr;
		buf[size1_idx] += siz_arr;
		buf[size2_idx] += siz_arr;
	}

	/* Copy footer */
	memcpy(buf+idx, ts_footer, sizeof(ts_footer));
	idx += sizeof(ts_footer);

	buf[idx++] = action;

	/* Send touch event */
	aa_cmd_send (idx, buf, 0, NULL);

	free(buf);
}

gboolean sdl_poll_event(gpointer data)
{
	SDL_Event event;
	SDL_MouseButtonEvent *mbevent;
	gst_app_t *app = (gst_app_t *)data;

	if (SDL_PollEvent(&event) >= 0) {
		switch (event.type) {
		case SDL_MOUSEBUTTONDOWN:
			mbevent = &event.button;
			if (mbevent->button == SDL_BUTTON_LEFT) {
//				printf("Left button down at x: %d y: %d\n", (int)((float)mbevent->x / 1024 * 800), (int)((float)mbevent->y / 600 * 480));
				aa_touch_event(ACTION_DOWN, (int)((float)mbevent->x / 1024 * 800), (int)((float)mbevent->y / 600 * 480));
			}
			break;
		case SDL_MOUSEBUTTONUP:
			mbevent = &event.button;
			if (mbevent->button == SDL_BUTTON_LEFT) {
//				printf("Left button up at x: %d y: %d\n", (int)((float)mbevent->x / 1024 * 800), (int)((float)mbevent->y / 600 * 480));
				aa_touch_event(ACTION_UP, (int)((float)mbevent->x / 1024 * 800), (int)((float)mbevent->y / 600 * 480));
			} else if (mbevent->button == SDL_BUTTON_RIGHT) {
				printf("Quitting...\n");
				SDL_Quit();
				g_main_loop_quit(app->loop);
				return FALSE;
			}
			break;
		case SDL_QUIT:
			printf("Quitting...\n");
			SDL_Quit();
			g_main_loop_quit(app->loop);
			return FALSE;
			break;
		}
	}

	return TRUE;
}

static int gst_loop(gst_app_t *app)
{
	int ret;
	GstStateChangeReturn state_ret;

	state_ret = gst_element_set_state((GstElement*)app->pipeline, GST_STATE_PLAYING);
//	g_warning("set state returned %d\n", state_ret);

	app->loop = g_main_loop_new (NULL, FALSE);
	g_timeout_add_full(G_PRIORITY_HIGH, 100, sdl_poll_event, (gpointer)app, NULL);
	printf("Starting Android Auto...\n");
  	g_main_loop_run (app->loop);

	/* Should not reach this? */
	SDL_Quit();

	state_ret = gst_element_set_state((GstElement*)app->pipeline, GST_STATE_NULL);
//	g_warning("set state null returned %d\n", state_ret);

	gst_object_unref(app->pipeline);

	return ret;
}

/* XPM */
static const char *arrow[] = {
	/* width height num_colors chars_per_pixel */
	"    32    32        3            1",
	/* colors */
	"X c #000000",
	". c #ffffff",
	"  c None",
	/* pixels */
	"X                               ",
	"XX                              ",
	"X.X                             ",
	"X..X                            ",
	"X...X                           ",
	"X....X                          ",
	"X.....X                         ",
	"X......X                        ",
	"X.......X                       ",
	"X........X                      ",
	"X.....XXXXX                     ",
	"X..X..X                         ",
	"X.X X..X                        ",
	"XX  X..X                        ",
	"X    X..X                       ",
	"     X..X                       ",
	"      X..X                      ",
	"      X..X                      ",
	"       XX                       ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"                                ",
	"0,0"
};

static SDL_Cursor *init_system_cursor(const char *image[])
{
	int i, row, col;
	Uint8 data[4*32];
	Uint8 mask[4*32];
	int hot_x, hot_y;

	i = -1;
	for ( row=0; row<32; ++row ) {
		for ( col=0; col<32; ++col ) {
			if ( col % 8 ) {
				data[i] <<= 1;
				mask[i] <<= 1;
			} else {
				++i;
				data[i] = mask[i] = 0;
			}
			switch (image[4+row][col]) {
				case 'X':
					data[i] |= 0x01;
					mask[i] |= 0x01;
					break;
				case '.':
					mask[i] |= 0x01;
					break;
				case ' ':
					break;
			}
		}
	}
	sscanf(image[4+row], "%d,%d", &hot_x, &hot_y);
	return SDL_CreateCursor(data, mask, 32, 32, hot_x, hot_y);
}

int main (int argc, char *argv[])
{
	gst_app_t *app = &gst_app;
	int ret = 0;
	errno = 0;
	byte ep_in_addr  = -1;
	byte ep_out_addr = -1;
	SDL_Cursor *cursor;

	/* Init gstreamer pipelien */
	ret = gst_pipeline_init(app);
	if (ret < 0) {
		printf("gst_pipeline_init() ret: %d\n", ret);
		return (ret);
	}

#if 0
	/* Overlay gst sink on the Qt window */
	WId xwinid = window->winId();
	gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(app->sink), xwinid);
#endif

	/* Start AA processing */
	ret = hu_aap_start (ep_in_addr, ep_out_addr);
	if (ret < 0) {
		if (ret == -2)
			printf("Phone is not connected. Connect a supported phone and restart.\n");
		else if (ret == -1)
			printf("Phone switched to accessory mode. Restart to enter AA mode.\n");
		else
			printf("hu_app_start() ret: %d\n", ret);
		return (ret);
	}

	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_WM_SetCaption("Android Auto", NULL);
	SDL_SetVideoMode(1024, 600, 16, SDL_HWSURFACE);
	cursor = init_system_cursor(arrow);
	SDL_SetCursor(cursor);
	SDL_ShowCursor(SDL_ENABLE);

	/* Start gstreamer pipeline and main loop */
	ret = gst_loop(app);
	if (ret < 0) {
		printf("gst_loop() ret: %d\n", ret);
	}

	SDL_Quit();

	/* Stop AA processing */
	ret = hu_aap_stop ();
	if (ret < 0) {
		printf("hu_aap_stop() ret: %d\n", ret);
		return (ret);
	}

	return (ret);
}
