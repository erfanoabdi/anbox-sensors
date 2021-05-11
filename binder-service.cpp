#include <gbinder.h>

#include <gutil_log.h>

#include <glib-unix.h>

#define BINDER_TRANSACTION(c2,c3,c4) GBINDER_FOURCC('_',c2,c3,c4)
#define BINDER_DUMP_TRANSACTION      BINDER_TRANSACTION('D','M','P')

#define RET_OK          (0)
#define RET_NOTFOUND    (1)
#define RET_INVARG      (2)
#define RET_ERR         (3)

#define DEFAULT_DEVICE  "/dev/binder"
#define DEFAULT_NAME    "test"
#define DEFAULT_IFACE   "test@1.0"

typedef struct app_options {
    char* dev;
    char* iface;
    const char* name;
    gboolean async;
} AppOptions;

typedef struct app {
    const AppOptions* opt;
    GMainLoop* loop;
    GBinderServiceManager* sm;
    GBinderLocalObject* obj;
    int ret;
} App;

typedef struct response {
    GBinderRemoteRequest* req;
    GBinderLocalReply* reply;
} Response;

static const char pname[] = "binder-service";

static
gboolean
app_signal(
    gpointer user_data)
{
    App* app = (App*) user_data;

    GINFO("Caught signal, shutting down...");
    g_main_loop_quit(app->loop);
    return G_SOURCE_CONTINUE;
}

static
gboolean
app_async_resp(
    gpointer user_data)
{
    Response* resp = (Response*) user_data;

    gbinder_remote_request_complete(resp->req, resp->reply, 0);
    return G_SOURCE_REMOVE;
}

static
void
app_async_free(
    gpointer user_data)
{
    Response* resp = (Response*) user_data;

    gbinder_local_reply_unref(resp->reply);
    gbinder_remote_request_unref(resp->req);
    g_free(resp);
}

static
GBinderLocalReply*
app_reply(
    GBinderLocalObject* obj,
    GBinderRemoteRequest* req,
    guint code,
    guint flags,
    int* status,
    void* user_data)
{
    App* app = (App*) user_data;
    GBinderReader reader;

    gbinder_remote_request_init_reader(req, &reader);
    if (code == GBINDER_FIRST_CALL_TRANSACTION) {
        const AppOptions* opt = app->opt;
        const char* iface = gbinder_remote_request_interface(req);

        if (!g_strcmp0(iface, opt->iface)) {
            char* str = gbinder_reader_read_string16(&reader);
            GBinderLocalReply* reply = gbinder_local_object_new_reply(obj);

            GVERBOSE("\"%s\" %u", iface, code);
            GDEBUG("\"%s\"", str);
            *status = 0;
            gbinder_local_reply_append_string16(reply, str);
            g_free(str);
            if (opt->async) {
                Response* resp = g_new0(Response, 1);

                resp->reply = reply;
                resp->req = gbinder_remote_request_ref(req);
                g_idle_add_full(G_PRIORITY_DEFAULT_IDLE, app_async_resp,
                    resp, app_async_free);
                gbinder_remote_request_block(resp->req);
                return NULL;
            } else {
                return reply;
            }
        } else {
             GDEBUG("Unexpected interface \"%s\"", iface);
        }
    } else if (code == BINDER_DUMP_TRANSACTION) {
        int fd = gbinder_reader_read_fd(&reader);
        const char* dump = "Sorry, I've got nothing to dump...\n";
        const gssize dump_len = strlen(dump);

        GDEBUG("Dump request from %d", gbinder_remote_request_sender_pid(req));
        if (write(fd, dump, dump_len) != dump_len) {
            GERR("Failed to write dump: %s", strerror(errno));
        }
        *status = 0;
        return NULL;
    }
    *status = -1;
    return NULL;
}

static
void
app_add_service_done(
    GBinderServiceManager* sm,
    int status,
    void* user_data)
{
    App* app = (App*) user_data;

    if (status == GBINDER_STATUS_OK) {
        printf("Added \"%s\"\n", app->opt->name);
        app->ret = RET_OK;
    } else {
        GERR("Failed to add \"%s\" (%d)", app->opt->name, status);
        g_main_loop_quit(app->loop);
    }
}

static
void
app_sm_presence_handler(
    GBinderServiceManager* sm,
    void* user_data)
{
    App* app = (App*) user_data;

    if (gbinder_servicemanager_is_present(app->sm)) {
        GINFO("Service manager has reappeared");
        gbinder_servicemanager_add_service(app->sm, app->opt->name, app->obj,
            app_add_service_done, app);
    } else {
        GINFO("Service manager has died");
    }
}

static
void
app_run(
   App* app)
{
    const char* name = app->opt->name;
    guint sigtrm = g_unix_signal_add(SIGTERM, app_signal, app);
    guint sigint = g_unix_signal_add(SIGINT, app_signal, app);
    gulong presence_id = gbinder_servicemanager_add_presence_handler
        (app->sm, app_sm_presence_handler, app);

    app->loop = g_main_loop_new(NULL, TRUE);

    gbinder_servicemanager_add_service(app->sm, name, app->obj,
        app_add_service_done, app);

    g_main_loop_run(app->loop);

    if (sigtrm) g_source_remove(sigtrm);
    if (sigint) g_source_remove(sigint);
    gbinder_servicemanager_remove_handler(app->sm, presence_id);
    g_main_loop_unref(app->loop);
    app->loop = NULL;
}

int main(int argc, char* argv[])
{
    App app;
    AppOptions opt;

    memset(&app, 0, sizeof(app));
    app.ret = RET_INVARG;
    app.opt = &opt;
    opt.dev = g_strdup(DEFAULT_DEVICE);
    opt.iface = g_strdup(DEFAULT_IFACE);
    opt.name = DEFAULT_NAME;
    gutil_log_timestamp = FALSE;
    gutil_log_set_type(GLOG_TYPE_STDERR, pname);
    gutil_log_default.level = GLOG_LEVEL_DEFAULT;

    app.sm = gbinder_servicemanager_new(opt.dev);
    if (gbinder_servicemanager_wait(app.sm, -1)) {
        app.obj = gbinder_servicemanager_new_local_object
            (app.sm, opt.iface, app_reply, &app);
        app_run(&app);
        gbinder_local_object_unref(app.obj);
        gbinder_servicemanager_unref(app.sm);
    }
    g_free(opt.iface);
    g_free(opt.dev);
    return app.ret;
}
