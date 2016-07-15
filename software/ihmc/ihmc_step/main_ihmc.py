import argparse
from ihmc_step.translator_ihmc import IHMCStepTranslator, Mode


def run_translator():
    t = IHMCStepTranslator(mode=Mode.translating, safe=False)
    t.run()

def run_plotter():
    t = IHMCStepTranslator(mode=Mode.plotting, safe=False)
    t.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="IHMC VERSION. Run the translation module which converts footstep plans into the spec required by the BDI controller")
    parser.add_argument('--base', '-b', dest='base', action='store_true', default=False, help='Run as the base-side translator, which plots expected swing trajectories and sends COMMITTED_FOOTSTEP_PLAN messages to the robot-side translator')
    args = parser.parse_args()
    if args.base:
        run_plotter()
    else:
        run_translator()
