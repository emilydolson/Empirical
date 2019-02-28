/**
 *  @note This file is part of Empirical, https://github.com/devosoft/Empirical
 *  @copyright Copyright (C) Michigan State University, MIT Software license; see doc/LICENSE.md
 *  @date 2018
 *
 *  @file  Input.h
 *  @brief Create/control an HTML input and call a specified function when it recieves input.
 *
 *  Use example:
 *
 *    emp::web::Input my_input(MyFun, "input type", "input Name", "html_id");
 *
 *  Where my_input is the C++ object linking to the input, MyFun is the
 *  function you want to call on chnages, "Input Name" is the label on the
 *  input itself, and "html_id" is the optional id you want it to use in the
 *  HTML code (otherwise it will generate a unique name on its own.)
 *
 *  Member functions to set state:
 *    Input & Callback(const std::function<void()> & in_callback)
 *    Input & Label(const std::string & in_label)
 *    Input & Autofocus(bool in_af)
 *    Input & Disabled(bool in_dis)
 *
 *  Retriving current state:
 *    const std::string & GetLabel() const
 *    bool HasAutofocus() const
 *    bool IsDisabled() const
 */

#ifndef EMP_WEB_Input_H
#define EMP_WEB_Input_H


#include "Widget.h"

namespace emp {
namespace web {

  /// Create or control an HTML Input object that you can manipulate and update as needed.
  class Input : public internal::WidgetFacet<Input> {
    friend class InputInfo;
  protected:

    // Inputs associated with the same DOM element share a single InputInfo object.
    class InputInfo : public internal::WidgetInfo {
      friend Input;
    protected:
      std::string label;
      std::string type;
      std::string min = "";
      std::string max = "";
      std::string value = "";
      std::string step = "";

      std::string curr_val ="";

      bool autofocus;

      std::function<void(std::string)> callback;
      uint32_t callback_id;
      std::string onchange_info;

      InputInfo(const std::string & in_id="") : internal::WidgetInfo(in_id) { ; }
      InputInfo(const InputInfo &) = delete;               // No copies of INFO allowed
      InputInfo & operator=(const InputInfo &) = delete;   // No copies of INFO allowed
      virtual ~InputInfo() {
        if (callback_id) emp::JSDelete(callback_id);         // Delete callback wrapper.
      }

      std::string GetTypeName() const override { return "InputInfo"; }

      virtual void GetHTML(std::stringstream & HTML) override {
        HTML.str("");                                           // Clear the current text.
        HTML << "<input type=\"" << type << "\"";               // Indicate input type.
        if (min != "") HTML << " min=\"" << min << "\"";        // Add a min allowed value if there is one.
        if (max != "") HTML << " max=\"" << max << "\"";        // Add a max allowed value if there is one.
        if (value != "") HTML << " value=\"" << value << "\"";  // Add a current value if there is one.
        if (step != "") HTML << " step=\"" << step << "\"";     // Add a step if there is one.
        HTML << " id=\"" << id << "\"";                         // Indicate ID.
        HTML << " onchange=\"" << onchange_info << "\"";        // Indicate action on change.
        HTML << ">" << label << "</input>";                     // Close and label the Input.
      }

      void DoChange(std::string new_val) {
        curr_val = new_val;
        callback(curr_val);
        UpdateDependants();
      }

      void UpdateCallback(const std::function<void(std::string)> & in_cb) {
        callback = in_cb;
      }

      void UpdateLabel(const std::string & in_label) {
        label = in_label;
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateType(const std::string & in_type) {
        type = in_type;
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateMin(const std::string & in_min) {
        min = in_min;
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateMin(const double & in_min) {
        min = to_string(in_min);
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateMax(const std::string & in_max) {
        max = in_max;
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateMax(const double & in_max) {
        max = to_string(in_max);
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateValue(const std::string & in_value) {
        value = in_value;
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateValue(const double & in_value) {
        value = to_string(in_value);
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateStep(const std::string & in_step) {
        step = in_step;
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateStep(const double & in_step) {
        step = to_string(in_step);
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateAutofocus(bool in_af) {
        autofocus = in_af;
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }
      void UpdateDisabled(bool in_dis) {
        if (in_dis) extras.SetAttr("disabled", "true");
        else extras.RemoveAttr("disabled");
        if (state == Widget::ACTIVE) ReplaceHTML();     // If node is active, immediately redraw!
      }

    public:
      virtual std::string GetType() override { return "web::InputInfo"; }
    }; // End of InputInfo definition


    // Get a properly cast version of indo.
    InputInfo * Info() { return (InputInfo *) info; }
    const InputInfo * Info() const { return (InputInfo *) info; }

    Input(InputInfo * in_info) : WidgetFacet(in_info) { ; }

  public:

    /// Create a new Input.
    /// @param in_cb The function to call when the Input is changed.
    /// @param in_type The type of this input.
    /// @param in_label The label that should appear on the Input.
    /// @param in_id The HTML ID to use for this Input (leave blank for auto-generated)
    Input(const std::function<void(std::string)> & in_cb, const std::string & in_type,
          const std::string & in_label, const std::string & in_id="")
      : WidgetFacet(in_id)
    {
      info = new InputInfo(in_id);

      Info()->label = in_label;
      Info()->type = in_type;
      Info()->autofocus = false;
      Info()->curr_val = "";

      Info()->callback = in_cb;
      InputInfo * b_info = Info();
      Info()->callback_id = JSWrap( std::function<void(std::string)>( [b_info](std::string new_val){b_info->DoChange(new_val);} )  );
      Info()->onchange_info = emp::to_string("emp.Callback(", Info()->callback_id, ", ('checked' in this) ? this.checked.toString() : this.value);");
    }

    /// Link to an existing Input.
    Input(const Input & in) : WidgetFacet(in) { ; }
    Input(const Widget & in) : WidgetFacet(in) { emp_assert(in.IsInput()); }
    Input() : WidgetFacet("") { info = nullptr; }
    virtual ~Input() { ; }

    using INFO_TYPE = InputInfo;

    /// Set a new callback function to trigger when the Input is clicked.
    Input & Callback(const std::function<void(std::string)> & in_cb) {
      Info()->UpdateCallback(in_cb);
      return *this;
    }

    /// Set a new label to appear on this Input.
    Input & Label(const std::string & in_label) { Info()->UpdateLabel(in_label); return *this; }

    /// Update the type
    Input & Type(const std::string & in_t) { Info()->UpdateType(in_t); return *this; }

    /// Update the min
    Input & Min(const std::string & in_m) { Info()->UpdateMin(in_m); return *this; }
    Input & Min(const double & in_m) { Info()->UpdateMin(in_m); return *this; }

    /// Update the max
    Input & Max(const std::string & in_m) { Info()->UpdateMax(in_m); return *this; }
    Input & Max(const double & in_m) { Info()->UpdateMax(in_m); return *this; }

    /// Update the current value
    Input & Value(const std::string & in_m) { Info()->UpdateValue(in_m); return *this; }
    Input & Value(const double & in_m) { Info()->UpdateValue(in_m); return *this; }

    /// Update the current step size
    Input & Step(const std::string & in_m) { Info()->UpdateStep(in_m); return *this; }
    Input & Step(const double & in_m) { Info()->UpdateStep(in_m); return *this; }

    /// Setup this Input to have autofocus (or remove it!)
    Input & Autofocus(bool in_af=true) { Info()->UpdateAutofocus(in_af); return *this; }

    /// Setup this Input to be disabled (or re-enable it!)
    Input & Disabled(bool in_dis=true) { Info()->UpdateDisabled(in_dis); return *this; }

    /// Get the current label on this Input.
    const std::string & GetCurrValue() const { return Info()->curr_val; }

    /// Get the current label on this Input.
    const std::string & GetLabel() const { return Info()->label; }

    /// Get the current type of this input.
    const std::string & GetType() const { return Info()->type; }

    /// Get the current min of this input.
    const std::string & GetMin() const { return Info()->min; }

    /// Get the current max of this input.
    const std::string & GetMax() const { return Info()->max; }

    /// Get the value attribute of this input.
    const std::string & GetValue() const { return Info()->value; }

    /// Get the value attribute of this input.
    const std::string & GetStep() const { return Info()->step; }

    /// Determine if this Input currently has autofocus.
    bool HasAutofocus() const { return Info()->autofocus; }

    /// Determine if this Input is currently disabled.
    bool IsDisabled() const { return Info()->extras.HasAttr("disabled"); }
  };


}
}

#endif
