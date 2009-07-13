#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/to_python_converter.hpp>

namespace sandbox { namespace {

  class custom_string
  {
    public:
      custom_string() {}
      custom_string(std::string const& value) : value_(value) {}
      std::string const& value() const { return value_; }
    private:
      std::string value_;
  };

  struct custom_string_to_python_str
  {
    static PyObject* convert(custom_string const& s)
    {
      return boost::python::incref(boost::python::object(s.value()).ptr());
    }
  };

  struct custom_string_from_python_str
  {
    custom_string_from_python_str()
    {
      boost::python::converter::registry::push_back(
        &convertible,
        &construct,
        boost::python::type_id<custom_string>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
      if (!PyString_Check(obj_ptr)) return 0;
      return obj_ptr;
    }

    static void construct(
      PyObject* obj_ptr,
      boost::python::converter::rvalue_from_python_stage1_data* data)
    {
      const char* value = PyString_AsString(obj_ptr);
      if (value == 0) boost::python::throw_error_already_set();
      void* storage = (
        (boost::python::converter::rvalue_from_python_storage<custom_string>*)
          data)->storage.bytes;
      new (storage) custom_string(value);
      data->convertible = storage;
    }
  };

  custom_string hello() { return custom_string("Hello world."); }

  std::size_t size(custom_string const& s) { return s.value().size(); }

  void init_module()
  {
    using namespace boost::python;

    boost::python::to_python_converter<
      custom_string,
      custom_string_to_python_str>();

    custom_string_from_python_str();

    def("hello", hello);
    def("size", size);
  }

}} // namespace sandbox::<anonymous>

BOOST_PYTHON_MODULE(custom_string)
{
  sandbox::init_module();
}
