#include <vector>
#include <cstdint>
#include <any>
#include <typeinfo>
#include <typeindex>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <cassert>
#include <functional>
#include <tuple>

#include <iostream>

#include "../objects/mass_particle.hpp"

namespace systems {
	struct particle_system {
	public:
		std::vector<std::vector<uint8_t>> particle_data;
		std::vector<std::vector<bool>> particle_data_mask;

		typedef size_t particle_id;
		typedef size_t attribute_id;
		std::unordered_map<std::type_index, attribute_id> attribute_map;

	public:
		particle_system() noexcept;
		particle_system(size_t particle_capacity, size_t attribute_capacity) noexcept;
		//~particle_system();

	private:
		template <class Map, class Key>
		static typename Map::mapped_type& get_or_insert(Map& m, Key const& key, typename Map::mapped_type const& value) {
			return m.insert(typename Map::value_type(key, value)).first->second;
		}

		template <typename T>
		static inline size_t particle_data_index(particle_id particle)  {
			return (sizeof(T) / sizeof(uint8_t)) * particle;
		}

		template <typename T>
		attribute_id get_or_register_attribute() {
			attribute_id inserted = get_or_insert(attribute_map, typeid(T), particle_data.size());
			if (inserted == particle_data.size()) {
				particle_data.push_back(std::vector<uint8_t>());
				particle_data_mask.push_back(std::vector<bool>());
			}

			std::cout << "getting attribute " << inserted << std::endl;
			return inserted;
		}

	public:
		template <typename T>
		bool particle_has_attribute(particle_id particle) {
			std::vector<bool> const mask = particle_data_mask.at(get_or_register_attribute<T>());
			return particle < mask.size() && mask.at(particle);
		}

		template <typename T>
		T& get_particle_attribute(particle_id particle) {
			assert(particle_has_attribute<T>(particle) && "error: attempted to access non-existing attribute of particle");
			return reinterpret_cast<T&>(particle_data.at(get_or_register_attribute<T>()).at(particle_data_index<T>(particle)));
		}

		template <typename T>
		T* get_particle_attribute_ptr(particle_id particle) {
			if (particle_has_attribute<T>(particle)) {
				return &get_particle_attribute<T>(particle);
			} else {
				return static_cast<T*>(nullptr);
			}
		}

		template <typename T>
		T *set_particle_attribute(particle_id particle, T&& attribute) {
			attribute_id attribute_id = get_or_register_attribute<T>();
			std::vector<bool> &mask = particle_data_mask.at(attribute_id);
			std::vector<uint8_t> &data = particle_data.at(attribute_id);

			size_t const data_index = particle_data_index<T>(particle);
			size_t const data_size = data.size();

			bool particle_in_mask = particle < mask.size();
			bool particle_in_data = data_index + sizeof(T) <= data_size;
			if (particle_in_mask != particle_in_data) {
				assert(false && "unreachable: attribute data and mask mismatch");
				std::exit(EXIT_FAILURE);
			}

			if (!particle_in_mask) {
				mask.resize(particle - mask.size() + 1);
			}
			mask.at(particle) = true;

			if (!particle_in_data) {
				size_t const missing_bytes = data_index + sizeof(T) - data_size;
				data.resize(data_size + missing_bytes);
			}

			return static_cast<T *>(std::memcpy(&data.at(data_index), &attribute, sizeof(T)));
		}

		template <typename T>
		bool remove_particle_attribute(particle_id particle, T &out_particle_attribute) {
			attribute_id const attribute_id = get_or_register_attribute<T>();
			if (!particle_has_attribute<T>(particle))
				return false;

			particle_data_mask.at(attribute_id).at(particle) = false;
			out_particle_attribute = reinterpret_cast<T&>(particle_data.at(attribute_id).at(
				particle_data_index<T>(particle)
			));
			return true;
		}

	private:
		template<typename Function, typename Tuple, size_t ... I>
		auto call(Function f, Tuple t, std::index_sequence<I ...>)
		{
			return f(std::get<I>(t) ...);
		}

		template<typename Function, typename Tuple>
		auto call(Function f, Tuple t)
		{
			static constexpr auto size = std::tuple_size<Tuple>::value;
			return call(f, t, std::make_index_sequence<size>{});
		}

		template<typename ...T, size_t... I>
		auto make_references(std::tuple<T...>& t, std::index_sequence<I...>) {
			return std::tie(*std::get<I>(t)...);
		}

		template<typename ...T>
		auto make_references(std::tuple<T...>& t) {
			return make_references<T...>(t, std::make_index_sequence<sizeof...(T)>{});
		}

	public:
		typedef size_t particle_count;

		inline particle_count count() const {
			return particle_data_mask.size() > 0 ? particle_data_mask.at(0).size() : 0;
		}

		template <typename ...Ts>	
		particle_count iter(std::function<void(Ts& ...)> const &func) {
			particle_count count = 0;
			for (size_t i = 0; i < this->count(); ++i) {
				bool all = true;
				std::tuple<Ts* ...> attributes = {
					[&]() {
						Ts* ptr = get_particle_attribute_ptr<Ts>(i);
						all &= ptr != nullptr;
						return ptr;
					}()...
				};

				if (all) {
					++count;
					call(func, make_references(attributes));
				}
			}

			return count;
		}

		//particle_count iter(std::function<void(int, float)> const& func) {
		//	particle_count count = 0;
		//	for (size_t i = 0; i < this->count(); ++i) {
		//		std::tuple<int, float> attributes;
		//		bool all = true;

		//		size_t j = 0;
		//		while (j < 2 && all)
		//		{
		//			if (all &= particle_has_attribute<int>(i)) {
		//				std::get<int>(attributes) = get_particle_attribute<int>(i);
		//			}
		//			++j;
		//		}

		//		if (all) {
		//			++count;
		//			call(func, attributes);
		//		}
		//	}

		//	return count;
		//}

		//template <typename ...Ts>
		//particle_count iter(void (* func)(Ts ...)) {
		//	std::function<void(Ts ...)> f = func;
		//	return iter(f);
		//}


		//struct early_exit {};
		//template<typename T>struct get {
		//	T operator()(std::size_t)const {
		//		throw early_exit();
		//	}
		//};
		//template<>struct get<int&> {
		//	int& operator()(std::size_t)const {
		//		static int x = 1;
		//		return x;
		//	}
		//};
		//template<>struct get<const float&> {
		//	const float& operator()(std::size_t)const {
		//		static const float x = 2;
		//		return x;
		//	}
		//};
		//template<>struct get<std::size_t> {
		//	std::size_t operator()(std::size_t x)const {
		//		return x == 1 ? throw early_exit() : x + 1;
		//	}
		//};
		//template<typename...Ts, std::size_t...Idxs>void call(std::function<void(Ts...)>const& func, std::tuple<Ts...>&& tup, std::index_sequence<Idxs...>) {
		//	func(std::get<Idxs>(std::move(tup))...);
		//}
		//template<typename...Ts>std::size_t foo(std::function<void(Ts...)>const& func, std::size_t count) {
		//	std::size_t result = 0;
		//	for (std::size_t i = 0; i < count; ++i) {
		//		try {
		//			call(func, std::tuple<Ts...>(get<Ts>()(i)...), std::make_index_sequence<sizeof...(Ts)>());
		//			++result;
		//		}
		//		catch (const early_exit&) {}
		//	}
		//	return result;
		//}
		//void test(int& x, const float& y, std::size_t z) {
		//	std::cout << x << ' ' << y << ' ' << z << '\n';
		//}
	};
}