from pywinauto.application import Application
from pywinauto import Desktop, timings
import time
import logging
import os
import pathlib
import collections
import csv

logger = logging.getLogger(__name__)
method_name = "__pywinauto_method__"
reuslt_folder_name = "pywinauto" # folder extension from base result folder
main_window = None
recovery_attempts = 5

app = None

ScanConfiguration = collections.namedtuple(
    "ScanConfiguration", ["SampleScans", "Resolution", "LowerWavenumber", "UpperWavenumber"]
)
default_scan_configuration = ScanConfiguration(
    SampleScans="5", Resolution="16", LowerWavenumber="400", UpperWavenumber="4000"
)

def start_and_login(recovery_attempt = 0):
    global app, main_window

    # Start the application 
    
    try:
        app = Application(backend="uia").start(r"C:\Program Files (x86)\Agilent\MicroLab PC\MicroLabPC.exe")

        logger.info("MicroLab PC starting...")

        # Wait for main window to appear
        main_window = app.window(title_re=".*MicroLab.*")  # Use regex if needed
        main_window.wait("visible", timeout=10)
        time.sleep(5)

    except Exception as e:
        print (e)
        if recovery_attempt < recovery_attempts:
            logger.error("Statup failed or timed out, retrying: %s/%s", (recovery_attempt+1),recovery_attempts)
            kill_app()
            return start_and_login(recovery_attempt+1)
        else:
            logger.error("Statup failed or timed out, exiting")
            kill_app()
            exit(-1)

    try:
        #Login
        logger.debug("Loggin to MicroLab PC")
        main_window.Edit.type_keys("admin")
        main_window.PasswordButton2.click_input() #login-button
        #Login complete 
        logger.info("login completed")

        check_home()
        return(main_window)

    except Exception as e:
        logger.error("Error interacting with UI: %s",e)
        app.kill()
        exit(-1)

def activate_method():
    global app, main_window
    main_window.set_focus()
    check_home()

    if main_window[method_name].exists():
        logger.debug("Method %s already selected", method_name)
        return (True)
    
    main_window.Button4.click_input() # Select method menu

    methodlistbox = main_window.listbox.wrapper_object() # wraper for better control of the list of methods
    # methodlistbox.scroll("down", "page")  

    methods = []
    for listitem in methodlistbox.items():
        methods += listitem.texts()
    logger.debug("Method items in list view: %s", methods)

    ### METHOD SUBMENU BUTTON MAPPINGS
    # main_window.Button14: home?
    # main_window.Button13: deleate method 
    # main_window.Button12: new method 
    # main_window.Button11: print 
    # main_window.Button10: edit
    # main_window.Button9: activate
    
    # Select method if it exists
    if method_name in methods:
        logger.debug("Selecting and activating method: %s", method_name)
        methodlistbox[method_name].select() # select method
        main_window.Button9.click_input() # Activate

        check_home()
        return(True)
    else:
        logger.warning("Failed to find method: %s", method_name)
        logger.warning("Method activation failed")
        main_window.Button14.click_input() # Home

        check_home()
        return(False)


def configure_scan(scan_configuration=default_scan_configuration):
    global app, main_window
    main_window.set_focus()
    check_home()

    ### HOME MENU BUTTON MAPPINGS
    # main_window.Button1: close?
    # main_window.Button2: minemize
    # main_window.Button3: refrence template
    # main_window.Button4: method
    # main_window.Button5: logoff
    # main_window.Button6: START

    main_window.Button4.click_input() # Select method menu

    methodlistbox = main_window.listbox.wrapper_object() # wraper for better control of the list of methods
    # methodlistbox.scroll("down", "page")  

    methods = []
    for listitem in methodlistbox.items():
        methods += listitem.texts()
    logger.debug("Method items in list view: %s", methods)

    ### METHOD SUBMENU BUTTON MAPPINGS
    # main_window.Button14: home?
    # main_window.Button13: deleate method 
    # main_window.Button12: new method 
    # main_window.Button11: print 
    # main_window.Button10: edit
    # main_window.Button9: activate
    
    # Delete old method (as there were problem when script saves method and then tries to edit again)
    if method_name in methods:
        logger.debug("Old method detected. Deleting method: %s", method_name)
        methodlistbox[method_name].select() # select method
        main_window.Button13.click_input() # Delete
        logger.debug("Deleate initiated: %s", method_name)
        main_window.ConfirmMethodDeletion.YesButton.click_input() # Confirm dialogue
        logger.debug("Deleate confirmed: %s", method_name)

    main_window.Button12.click_input() # new method

    ### Entering new method/edit submenu

    # Not needed but used by logger
    edittabcontroll = main_window.tabcontroll.wrapper_object()
    logger.debug("Edit view tab controll object: %s", edittabcontroll)
    logger.debug("Tabcontroll children: %s",edittabcontroll.children_texts())

    # Select data collection only
    main_window.type.methodTypeCombobox.wrapper_object().select("Data Collect Only")

    # Enable background Valid time 
    main_window.BackgroundValidTImeRadioButton.click_input()

    # Edit valid time
    main_window.MinuteComboBox.select("Day(s)")
    main_window.ReadyEdit.type_keys("1000") # Time edit for valid time

    # Enable clasic view
    main_window.EnableClassicView.click_input()

    # Turn of clean crystal check
    check_clean_checkbox = main_window.CheckforCleanCrystalPriortoCollectingBackground.wrapper_object()
    check_clean_checkbox_state = check_clean_checkbox.get_toggle_state()
    logger.debug("Check clean checkbox state before: %s", check_clean_checkbox_state)

    # WARING: One of the other script operations here toggle this option so i don't switch here 
    if check_clean_checkbox_state:
        check_clean_checkbox.click_input()

    check_clean_checkbox_state = check_clean_checkbox.get_toggle_state()
    logger.debug("Check clean checkbox state after: %s", check_clean_checkbox_state)

    # Select the instrument tab
    instrument_tab = main_window.instrument
    instrument_tab.select()
    logger.debug("Instrument tab children: %s",instrument_tab.children_texts())

    # Check if full spectrum is selected and if so unselect to unlock editing
    full_spectrum_checkbox = instrument_tab.FullCheckbox.wrapper_object()
    full_spectrum_checkbox_state = full_spectrum_checkbox.get_toggle_state()
    logger.debug("Spectral range - 'Full' checkbox state before: %s", full_spectrum_checkbox_state)

    if full_spectrum_checkbox_state:
        full_spectrum_checkbox.click_input()
    
    logger.debug("Spectral range - 'Full' checkbox state after: %s", full_spectrum_checkbox.get_toggle_state())

    # Edit spectral range
    main_window.SpectralRangeEdit.type_keys(str(scan_configuration.UpperWavenumber)) # High
    main_window.toEdit.type_keys(str(scan_configuration.LowerWavenumber))            # Low
    
    # Edit sample scans
    main_window.SampleScansEdit.type_keys(str(int(scan_configuration.SampleScans))) # Sample scans

    # Edit resolution
    resolution_combobox = main_window.resolution_combobox
    logger.debug("Resolution combobox object: %s", resolution_combobox)
    resolution_combobox.expand()
    resolution_combobox.select((str(int(scan_configuration.Resolution)))) # Select resolution
    if main_window.SamplingTecnologyListBox.exists() or main_window.ApodizationListBox.exists():
        logger.warning("Listview found when none expected during configure_scan")
    resolution_combobox.collapse()
    main_window.SamplingTecnologyComboBox.collapse()
    main_window.ApodizationListBox.collapse()

    ### EDIT SUBMENU BUTTON MAPPINGS (only tested from Instrument tab)
    # main_window.button3 : methods
    # main_window.button4 : save as
    # main_window.button5 : save
    # main_window.button6 : home

    # Save new method (as there were problems with editing)
    main_window.Button5.click_input() # SAVE
    
    if main_window.SaveAs.exists():
        logger.debug("Save as dialogue window identified")
        main_window.SaveAs.FilenameComboBox.Edit.type_keys(method_name) # Set method name
        logger.debug("Method name set: %s", method_name)
        main_window.SaveAs.SaveButton.click_input() # Confirm save   
        logger.debug("Method saved")

    # Select method and activate it
    methodlistbox[method_name].select() # select method
    main_window.button9.click_input() # Activate (selected method)
    ### Entering home menue
    check_home()

def run_background_scan():
    global app, main_window
    main_window.set_focus()
    logger.info("Running background scan")
    check_home()

    main_window.Button6.click_input() #Start

    ### BACKGROUNDSCAN SUBMENU BUTTON MAPPINGS (only tested from Instrument tab)
    # main_window.button4: home
    # main_window.button3: next

    if not main_window.PlaceSample.exists():
        logger.debug("Did not detect 'Place Sample' and thus runs background scan")
        main_window.Button3.click_input() # next

        main_window.CheckingTheCrystalStatic.wait_not("exists", timeout=10.5, retry_interval=.5) # wait untill next screen
        # main_window.print_control_identifiers()
        main_window.CollectingBackgroundStatic.wait_not("exists", timeout=10.5, retry_interval=.5) # wait untill next screen

        logger.debug("Backgroundscan complete")
        # main_window.print_control_identifiers()
    else:
        logger.warning("Background scan already ran for selected method")
    main_window.Button4.click_input() # Home
    check_home()

def run_scan():
    global app, main_window
    main_window.set_focus()
    logger.info("Running scan")
    main_window.Button6.click_input() # Start

    if not main_window.PlaceSample.exists():
        logger.error("Background scan not ran")
        main_window.Button4.click_input() # Home TODO: Test this row
        raise RuntimeError("Unexpected controll flow - background scan not ran before scan")
    
    main_window.Button3.click_input() # next

    ### TODO: Add sample name? Probably not neccecery
    # main_window.SampleEdit.type_keys(method_name) # not tested
    
    main_window.Button3.click_input() # next

    if main_window.InsufficientSpectralInformation.exists():
        logger.warning("Insufficient spectrum information detected")
        main_window.IgnoreButton.click_input() # ignore

    main_window.SamplingProgressPanel.wait_not("exists", timeout=30.5, retry_interval=.5) # Wait unti next screen

    ### Entering sample view
    # main_window.Button3: Done (take another sample)
    # main_window.Button4: Data handling
    # main_window.Button5: Home 

    main_window.Button4.click_input() # Data handling

    ### Entering data export view
    # main_window.Button3: Done
    # main_window.Button4: < Back ?
    # main_window.Button5: Export
    # main_window.Button6: Add to a library...
    # main_window.Button7: Report ?
    # main_window.Button8: Home

    main_window.Button5.click_input() # Export
    file_export_window = main_window.ExportFileTypeSelection
    
    file_export_window.SelectFileTypeComboBox.expand()
    file_export_window.SelectFileTypeComboBox.SelectFileTypeListBox.CommaSeparatedValueListItem.select() # select .csv
    file_export_window.SelectFileTypeComboBox.collapse()
    logger.debug("Selected filetype: %s", file_export_window.SelectFileTypeComboBox.selected_text())
    
    old_path = file_export_window.Edit1.get_value()
    result_path = pathlib.Path(old_path) # Current Location
    result_path = result_path / reuslt_folder_name # Add result folder 

    if not os.path.exists(result_path):
        logger.warning("No results folder identified. Making dir...")
        os.makedirs(result_path._)

    file_export_window.Edit1.set_text((str(result_path / " ")).replace(" ", "")) # Result sub dir location

    filename = file_export_window.Edit2.get_value()+".csv"
    logger.debug("Current filename for spectrum data: %s", filename)

    path_and_filename = result_path / (file_export_window.Edit2.get_value()+".csv") # add filename

    logger.debug("Scan path_and_filename: %s", path_and_filename)

    # file_export_window.print_control_identifiers()

    file_export_window.OKButton.click() #confirm WARN: NEEDS to be .click() instead of .click_input() to add some delay


    logger.debug("Data saved. Returning to home screen")
    main_window.Button8.click_input() # Home
    # main_window.print_control_identifiers()

    # return(str(path_and_filename).replace(("\\Public\\Documents\\"), ("\\Public\\Public Documents\\")))
    check_home()
    return(path_and_filename)

def kill_app():
    global app, main_window
    if app:
        app.kill()

def check_home():
    global app, main_window
    if not main_window.LogOffTheCurrentUserPane.exists():
        raise RuntimeError("Unexpected controll frame - main_window is not at home screen")

def main():
    # Initialize logger
    level=logging.DEBUG # WARNING, INFO, DEBUG
    logging.basicConfig(level=level, format="%(asctime)s : %(levelname)s : %(message)s")
    timeconfig = timings.TimeConfig
    timeconfig.fast(timeconfig)

    logger.info('Started ftir_pywinauto.py main method')
    try:
        ### Test with config
        start_and_login()
        configure_scan()
        run_background_scan()
        spectrum_data_path=run_scan()

        ### Test with just activate 
        # start_and_login()
        # activated = activate_method()
        # if not activated:
        #     configure_scan()
        # run_background_scan()
        # spectrum_data_path=run_scan()
        
        if logger.level == logging.DEBUG:
            folder = pathlib.Path(r'C:\Users\Public\Documents\Agilent\MicroLab\Results\pywinauto')
            logger.debug("Files in folder: %s", folder)
            for file in folder.glob('*'):
                logger.debug(file.name)

        
        logger.info("Spectrum data stored at: %s", spectrum_data_path)

        pathlib_spectrum_path = pathlib.Path(spectrum_data_path)
        logger.info("Spectrum data stored at: %s", pathlib_spectrum_path)
        logger.info("Spectrum data stored at exists: %s", pathlib_spectrum_path.exists())

        SpectrumPoint = collections.namedtuple("SpectrumPoint", ["Wavenumber", "Absorbance"])
        spectrum_data = []
        with open(spectrum_data_path, 'r', newline='') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)  # Skip header row
            for row in reader:
                if row: # ensure row is not empty
                    wavenumber = float(row[0])
                    absorbance = float(row[1])
                    spectrum_data.append(SpectrumPoint(Wavenumber=wavenumber, Absorbance=absorbance))

    except Exception as e:
        logger.error("Error interacting with UI: %s", e)
        kill_app()
        exit()

    logger.info('Finished ftir_pywinauto.py main method')


if __name__ == "__main__":
    main()