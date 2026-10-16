#include <gtk/gtk.h>
#include <stdatomic.h>
#include <stdio.h>

#include "balance_loop.h"
#include "gui.h"

extern atomic_bool running;

static GMainLoop *main_loop;
static GtkLabel *pitch_label;
static GtkLabel *motor_labels[2];
static GtkLabel *status_label;
static GtkCheckButton *enable_button;
static GtkAdjustment *gain_adjustments[3];
static gboolean updating_gains;

static void request_shutdown(void)
{
    balance_loop_enable(false);
    atomic_store(&running, false);
    g_main_loop_quit(main_loop);
}

static void on_quit(GtkButton *button, gpointer data)
{
    (void)button;
    (void)data;
    request_shutdown();
}

static gboolean on_close(GtkWindow *window, gpointer data)
{
    (void)window;
    (void)data;
    request_shutdown();
    return TRUE;
}

static void on_enable(GtkCheckButton *button, gpointer data)
{
    (void)data;
    balance_loop_enable(gtk_check_button_get_active(button));
}

static void on_gain_changed(GtkAdjustment *adjustment, gpointer data)
{
    (void)adjustment;
    (void)data;
    if (updating_gains) {
        return;
    }
    balance_loop_set_gains(
        gtk_adjustment_get_value(gain_adjustments[0]),
        gtk_adjustment_get_value(gain_adjustments[1]),
        gtk_adjustment_get_value(gain_adjustments[2])
    );
}

static void on_reset(GtkButton *button, gpointer data)
{
    (void)button;
    (void)data;
    gtk_check_button_set_active(enable_button, FALSE);
    balance_loop_enable(false);

    updating_gains = TRUE;
    gtk_adjustment_set_value(gain_adjustments[0], DEFAULT_KP);
    gtk_adjustment_set_value(gain_adjustments[1], DEFAULT_KI);
    gtk_adjustment_set_value(gain_adjustments[2], DEFAULT_KD);
    updating_gains = FALSE;
    balance_loop_set_gains(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
}

static gboolean update_display(gpointer data)
{
    (void)data;
    if (!atomic_load(&running)) {
        g_main_loop_quit(main_loop);
        return G_SOURCE_CONTINUE;
    }

    char text[128];
    snprintf(text, sizeof(text), "Raw pitch: %.2f deg (upright reference: 180 deg)",
             atomic_load(&g_pitch_deg));
    gtk_label_set_text(pitch_label, text);

    for (int motor = 0; motor < 2; ++motor) {
        snprintf(text, sizeof(text), "Motor %d: %.2f RPM | duty %+.3f", motor,
                 atomic_load(&g_motor_rpm[motor]), atomic_load(&g_duty[motor]));
        gtk_label_set_text(motor_labels[motor], text);
    }

    bool enabled = atomic_load(&g_enabled);
    if (!enabled && gtk_check_button_get_active(enable_button)) {
        gtk_check_button_set_active(enable_button, FALSE);
    }
    gtk_label_set_text(status_label,
        atomic_load(&g_fault) ? "FAULT: see terminal for cause."
        : enabled ? "Motor output enabled" : "Motor output disabled");
    return G_SOURCE_CONTINUE;
}

static void add_gain_row(GtkGrid *grid, int index, const char *label, double value)
{
    GtkAdjustment *adjustment = gtk_adjustment_new(value, 0.0, 1.0, 0.001, 0.01, 0.0);
    gain_adjustments[index] = adjustment;
    GtkWidget *scale = gtk_scale_new(GTK_ORIENTATION_HORIZONTAL, adjustment);
    GtkWidget *spin = gtk_spin_button_new(adjustment, 0.001, 3);
    gtk_scale_set_digits(GTK_SCALE(scale), 3);
    gtk_widget_set_hexpand(scale, TRUE);

    int row = index + 3;
    gtk_grid_attach(grid, gtk_label_new(label), 0, row, 1, 1);
    gtk_grid_attach(grid, scale, 1, row, 1, 1);
    gtk_grid_attach(grid, spin, 2, row, 1, 1);
    g_signal_connect(adjustment, "value-changed", G_CALLBACK(on_gain_changed), NULL);
}

void gui_init(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!gtk_init_check()) {
        fprintf(stderr, "Cannot open GTK display.\n");
        atomic_store(&running, false);
        return;
    }

    main_loop = g_main_loop_new(NULL, FALSE);
    GtkWidget *window = gtk_window_new();
    gtk_window_set_title(GTK_WINDOW(window), "Lab 05 - PID Balance Controller");
    gtk_window_set_default_size(GTK_WINDOW(window), 760, 450);

    GtkWidget *grid_widget = gtk_grid_new();
    GtkGrid *grid = GTK_GRID(grid_widget);
    gtk_grid_set_row_spacing(grid, 12);
    gtk_grid_set_column_spacing(grid, 14);
    gtk_widget_set_margin_start(grid_widget, 18);
    gtk_widget_set_margin_end(grid_widget, 18);
    gtk_widget_set_margin_top(grid_widget, 18);
    gtk_widget_set_margin_bottom(grid_widget, 18);
    gtk_window_set_child(GTK_WINDOW(window), grid_widget);

    pitch_label = GTK_LABEL(gtk_label_new("Pitch: waiting for sensor"));
    gtk_grid_attach(grid, GTK_WIDGET(pitch_label), 0, 0, 3, 1);
    for (int motor = 0; motor < 2; ++motor) {
        motor_labels[motor] = GTK_LABEL(gtk_label_new("Motor: waiting for sensor"));
        gtk_grid_attach(grid, GTK_WIDGET(motor_labels[motor]), 0, motor + 1, 3, 1);
    }

    double kp, ki, kd;
    balance_loop_get_gains(&kp, &ki, &kd);
    add_gain_row(grid, 0, "Kp (duty/deg)", kp);
    add_gain_row(grid, 1, "Ki (duty/(deg*s))", ki);
    add_gain_row(grid, 2, "Kd (duty*s/deg)", kd);

    enable_button = GTK_CHECK_BUTTON(gtk_check_button_new_with_label("Enable motor output"));
    gtk_grid_attach(grid, GTK_WIDGET(enable_button), 0, 6, 3, 1);
    g_signal_connect(enable_button, "toggled", G_CALLBACK(on_enable), NULL);

    status_label = GTK_LABEL(gtk_label_new("Motor output disabled"));
    gtk_label_set_wrap(status_label, TRUE);
    gtk_grid_attach(grid, GTK_WIDGET(status_label), 0, 7, 3, 1);

    GtkWidget *reset_button = gtk_button_new_with_label("Reset Gains");
    GtkWidget *quit_button = gtk_button_new_with_label("Quit");
    gtk_grid_attach(grid, reset_button, 0, 8, 1, 1);
    gtk_grid_attach(grid, quit_button, 2, 8, 1, 1);
    g_signal_connect(reset_button, "clicked", G_CALLBACK(on_reset), NULL);
    g_signal_connect(quit_button, "clicked", G_CALLBACK(on_quit), NULL);
    g_signal_connect(window, "close-request", G_CALLBACK(on_close), NULL);

    guint timer_id = g_timeout_add(50, update_display, NULL);
    gtk_window_present(GTK_WINDOW(window));
    g_main_loop_run(main_loop);

    g_source_remove(timer_id);
    gtk_window_destroy(GTK_WINDOW(window));
    g_main_loop_unref(main_loop);
}
