# Classes

```cpp
// Class definition
class Hobbit {
	public:
		string name;
		float height; //in meters
		int age;

		Hobbit();  // Constructor
		~Hobbit(); // Destructor

		void get_name() {
			cout << "This hobbit's name is " << name << endl;
		}
		void convert_height_to_cm();
	private:
		//
	protected:
		//
};

// Class method definitions
Hobbit::Hobbit() {
}
Hobbit::~Hobbit() {
}
void Hobbit::convert_height_to_cm() {
	height = height*100;
	cout << "Height of the hobbit is " << height << " centimeters" << endl;
}
```