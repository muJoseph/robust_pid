// Automatically generated C++ file on Thu Nov 12 17:37:50 2026
//
// To build with Digital Mars C++ Compiler:
//
//    dmc -mn -WD -o robust_pid.cpp kernel32.lib


/////////////////////////////////////////////////////////////////////
// Include
/////////////////////////////////////////////////////////////////////

#include <malloc.h>
#include <float.h>

extern "C" {
#include "robust_pid.h"
#include "robust_pid.c" // Must include source to prevent linker error
}

/////////////////////////////////////////////////////////////////////
// Extern functions
/////////////////////////////////////////////////////////////////////

extern "C" __declspec(dllexport) int (*Display)(const char *format, ...) = 0; // works like printf()
extern "C" __declspec(dllexport) const double *DegreesC                  = 0; // pointer to current circuit temperature


union uData
{
   bool b;
   char c;
   unsigned char uc;
   short s;
   unsigned short us;
   int i;
   unsigned int ui;
   float f;
   double d;
   long long int i64;
   unsigned long long int ui64;
   char *str;
   unsigned char *bytes;
};

// int DllMain() must exist and return 1 for a process to load the .DLL
// See https://docs.microsoft.com/en-us/windows/win32/dlls/dllmain for more information.
int __stdcall DllMain(void *module, unsigned int reason, void *reserved) { return 1; }


// memset
void bzero(void *ptr, unsigned int count)
{
   unsigned char *first = (unsigned char *) ptr;
   unsigned char *last  = first + count;
   while(first < last)
      *first++ = '\0';
}// bzero

static bool pid_isfinite(robust_pid_data_t x)
{
    // Both comparisons must pass.
    // NaN fails the comparisons; infinities exceed the bounds.
    return (x >= -FLT_MAX) && (x <= FLT_MAX);
}// pid_isfinite

// Initialize the PID context and its IIR coefficients.
//
// Ts_seconds:   Controller sample period, in seconds.
// cutoff_hz:    Derivative low-pass cutoff, in Hz.
//
// Returns false for a null context, invalid timing/filter parameters,
// or non-finite gains.
static bool init_pid(
    robust_pid_t *pid,
    robust_pid_data_t Ts_seconds,
    robust_pid_data_t cutoff_hz,
    robust_pid_data_t Kp,
    robust_pid_data_t Ki,
    robust_pid_data_t Kd)
{
    if (pid == 0 ||
        !pid_isfinite(Ts_seconds) || Ts_seconds <= 0.0f ||
        !pid_isfinite(cutoff_hz)  || cutoff_hz <= 0.0f ||
        !pid_isfinite(Kp) ||
        !pid_isfinite(Ki) ||
        !pid_isfinite(Kd))
    {
        return false;
    }

    robust_pid_cfg_t cfg;
    cfg.T          = Ts_seconds;
    cfg.LPF_3dB_Hz = cutoff_hz;
    cfg.Kp         = Kp;
    cfg.Ki         = Ki;
    cfg.Kd         = Kd;

    return robust_pid_initCtx(pid, &cfg);
}// init_pid

// #undef pin names lest they collide with names in any header file(s) you might include.
#undef r_set
#undef y_fb
#undef clk
#undef cntrl

typedef struct _robust_pid_ctx
{
  bool   clk_n1;      // clk[n-1], last clock state
  double error_n1;    // error[n-1]
  double errorI_n1;   // errorI[n-1]
  double errorD_n1;   // errorD[n-1]
  double lastT;
  robust_pid_t    pid;

}robust_pid_ctx_t;

extern "C" __declspec(dllexport) void robust_pid(robust_pid_ctx_t **opaque, double t, union uData *data)
{
   float  r_set   = data[0].f; // input
   float  y_fb    = data[1].f; // input
   bool   clk     = data[2].b; // input
   double Kp      = data[3].d; // input parameter
   double Ki      = data[4].d; // input parameter
   double Kd      = data[5].d; // input parameter
   double T       = data[6].d; // input parameter
   double LPF_3dB = data[7].d; // input parameter
   float &cntrl   = data[8].f; // output

   // Create/init local context
   if(!*opaque)
   {
      *opaque = (robust_pid_ctx_t *) malloc(sizeof(robust_pid_ctx_t));

      if(*opaque)
      {
         bzero(*opaque, sizeof(robust_pid_ctx_t));
         robust_pid_ctx_t *ctx = *opaque;

         // Init underlying robust_pid context
         const bool ok = init_pid(
            &ctx->pid,
            T,          // Ts
            LPF_3dB,    // Derivative cutoff
            Kp,         // Kp
            Ki,         // Ki
            Kd          // Kd
            );

         if( ok )
         {
            Display("robust_pid: Kp=%0.4f\n", Kp );
            Display("robust_pid: Ki=%0.4f\n", Ki );
            Display("robust_pid: Kd=%0.4f\n", Kd );
            Display("robust_pid: T=%0.4f, LPF Cutoff=%0.2f\n", T, LPF_3dB );
         }
         else
         {
            Display("robust_pid: Init failed!");
         }

      }

   }

   // Get context
   robust_pid_ctx_t *ctx = *opaque;

   // Rising edge trigger
   if (clk && !ctx->clk_n1)
   {
      // Run PID
      cntrl = robust_pid_run( &ctx->pid, r_set, y_fb );
   }

   // Update clock state for next invocation
   ctx->clk_n1 = clk;

}


/////////////////////////////////////////////////////////////////////
// Optional DLL Callbacks
/////////////////////////////////////////////////////////////////////
static const double DEFAULT_TTOL = 10e-12;  // 10 ps

// From help doc: MaxExtStepSize(), allows you to strategize a maximum time step given the
// state of your module and the simulation time.
extern "C" __declspec(dllexport) double MaxExtStepSize(robust_pid_ctx_t *inst)
{
   //return 1e308; // implement a good choice of max timestep size that depends on robust_pid_ctx_t
   return 1e-3;    // 1 ms
}// MaxExtStepSize


// From help doc: Trunc(), allows you to tactically reduce a specific timestep for the current state, input vector, simulation time.
// The boilerplate Trunc()implementation makes a copy of the instance data, calls the normal evaluation function,
// and then compares if anything changed. It allows QSPICE to do binary search for the maximum timestep that achieves
// the desired temporal tolerance.
extern "C" __declspec(dllexport) void Trunc(robust_pid_ctx_t *inst, double t, union uData *data, double *timestep)
{

   if(*timestep > DEFAULT_TTOL)
   {
      // Copy (dereference) robust_pid_ctx_t so it can be compared after robust_pid() invocation
      robust_pid_ctx_t temp = *inst;
      robust_pid(&(&temp), t, data);

      // Check if rising edge:
      // Limit the timestep to tolerance limit if the circuit causes a change in robust_pid_ctx_t
      // e.g. last clock state changed after robust_pid() invocation
      if( (temp.clk_n1 != inst->clk_n1) && !inst->clk_n1)
      {
         *timestep = DEFAULT_TTOL;    // clock rising edge, reduce timestep to TTOL
      }

   }
}// Trunc


extern "C" __declspec(dllexport) void Destroy(robust_pid_ctx_t *inst)
{
   free(inst);
}// Destroy
