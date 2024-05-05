// 26 june 2015
#include "uipriv_unix.h"

// LONGTERM figure out why, and describe, that this is the desired behavior
// LONGTERM also point out that font and color buttons also work like this

#define windowWindow(w) ((w) ? (GTK_WINDOW(uiControlHandle(uiControl(w)))) : NULL)

static char *filedialog(GtkWindow *parent, GtkFileChooserAction mode, const gchar *confirm, const uiFileTypeFilter *filters, int filters_len)
{
	GtkWidget *fcd;
	GtkFileChooser *fc;
	gint response;
	char *filename;

	fcd = gtk_file_chooser_dialog_new(NULL, parent, mode,
		"_Cancel", GTK_RESPONSE_CANCEL,
		confirm, GTK_RESPONSE_ACCEPT,
		NULL);
	fc = GTK_FILE_CHOOSER(fcd);
	gtk_file_chooser_set_local_only(fc, FALSE);
	gtk_file_chooser_set_select_multiple(fc, FALSE);
	gtk_file_chooser_set_show_hidden(fc, TRUE);
	gtk_file_chooser_set_do_overwrite_confirmation(fc, TRUE);
	gtk_file_chooser_set_create_folders(fc, TRUE);

	char *pattern = NULL;
	int pattern_cap = 0;
	for (int i = 0; i < filters_len; i++) {
		GtkFileFilter* gfilter = gtk_file_filter_new();
		gtk_file_filter_set_name(gfilter, filters[i].name);
		for (int j = 0; j < filters[i].extensions_len; j++) {
			int ext_len = strlen(filters[i].extensions[j]);
			if (3 + ext_len > pattern_cap) {
				pattern = uiprivRealloc(pattern, 3 + ext_len, "char[]");
				pattern_cap = 3 + ext_len;
			}
			sprintf(pattern, "*.%s", filters[i].extensions[j]);
			gtk_file_filter_add_pattern(gfilter, pattern);
			gtk_file_chooser_add_filter(fc, gfilter);
		}
	}
	if (pattern != NULL) uiprivFree(pattern);

	response = gtk_dialog_run(GTK_DIALOG(fcd));
	if (response != GTK_RESPONSE_ACCEPT) {
		gtk_widget_destroy(fcd);
		return NULL;
	}
	filename = gtk_file_chooser_get_filename(fc);
	gtk_widget_destroy(fcd);
	return filename;
}

char *uiOpenFile(uiWindow *parent)
{
	return filedialog(windowWindow(parent), GTK_FILE_CHOOSER_ACTION_OPEN, "_Open", NULL, 0);
}

char *uiOpenFolder(uiWindow *parent)
{
	return filedialog(windowWindow(parent), GTK_FILE_CHOOSER_ACTION_SELECT_FOLDER, "_Open", NULL, 0);
}

char *uiSaveFile(uiWindow *parent)
{
	return filedialog(windowWindow(parent), GTK_FILE_CHOOSER_ACTION_SAVE, "_Save", NULL, 0);
}

char *uiSaveFile2(uiWindow *parent, const uiFileTypeFilter *filters, int filters_len)
{
	return filedialog(windowWindow(parent), GTK_FILE_CHOOSER_ACTION_SAVE, "_Save", filters, filters_len);
}

static void msgbox(GtkWindow *parent, const char *title, const char *description, GtkMessageType type, GtkButtonsType buttons)
{
	GtkWidget *md;

	md = gtk_message_dialog_new(parent, GTK_DIALOG_MODAL,
		type, buttons,
		"%s", title);
	gtk_message_dialog_format_secondary_text(GTK_MESSAGE_DIALOG(md), "%s", description);
	gtk_dialog_run(GTK_DIALOG(md));
	gtk_widget_destroy(md);
}

void uiMsgBox(uiWindow *parent, const char *title, const char *description)
{
	msgbox(windowWindow(parent), title, description, GTK_MESSAGE_OTHER, GTK_BUTTONS_OK);
}

void uiMsgBoxError(uiWindow *parent, const char *title, const char *description)
{
	msgbox(windowWindow(parent), title, description, GTK_MESSAGE_ERROR, GTK_BUTTONS_OK);
}
