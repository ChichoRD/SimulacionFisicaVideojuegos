#ifndef PARTICLE_STORAGE_HPP
#define PARTICLE_STORAGE_HPP

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

namespace systems {
	typedef size_t particle_id;
	typedef size_t attribute_id;
	typedef size_t particle_count_t;
	typedef size_t attribute_count_t;

	struct attribute_storage {
	public:
		enum storage_type {
			VECTOR,
			UNIT,
			SHARED
		};

	public:
		size_t type_size;

		std::vector<bool> mask;
		std::vector<uint8_t> data;
		storage_type storage_variant;

	public:
		attribute_storage();
		attribute_storage(attribute_storage const &other);
		attribute_storage(size_t particle_capacity, storage_type storage_type, size_t type_size);

		attribute_storage(attribute_storage &&other);
		attribute_storage &operator=(attribute_storage &&other);

		template <typename T>
		static inline attribute_storage create_vector_storage(size_t particle_capacity) {
			return attribute_storage(particle_capacity, storage_type::VECTOR, sizeof(T));
		}

		static inline attribute_storage create_unit_storage(size_t particle_capacity) {
			return attribute_storage(particle_capacity, storage_type::UNIT, sizeof(bool));
		}

		template <typename T>
		static inline attribute_storage create_shared_storage(size_t particle_capacity) {
			return attribute_storage(particle_capacity, storage_type::SHARED, sizeof(T));
		}

	public:
		inline particle_count_t particle_count() const {
			return mask.size();
		}

		template <typename T>
		static constexpr size_t particle_data_index(particle_id particle)  {
			return (sizeof(T) / sizeof(uint8_t)) * particle;
		}

		static constexpr size_t particle_data_index(particle_id particle, size_t type_size) {
			return (type_size / sizeof(uint8_t)) * particle;
		}
	
	public:
		inline bool particle_has_attribute(particle_id particle) const {
			return particle < mask.size() && mask.at(particle);
		}

		void *get_particle_attribute_ptr(particle_id particle);

		inline void const *get_particle_attribute_const_ptr(particle_id particle) const {
			return static_cast<void const*>(
				const_cast<attribute_storage*>(this)->get_particle_attribute_ptr(particle)
			);
		}

		template <typename T>
		T& get_particle_attribute(particle_id particle) {
			return *static_cast<T*>(get_particle_attribute_ptr(particle));
		}

		template <typename T>
		T const& get_particle_attribute(particle_id particle) const {
			return get_particle_attribute<T>(particle);
		}


		void *set_particle_attribute_ptr(particle_id particle, void const * &&attribute);

		template <typename T>
		T &set_particle_attribute(particle_id particle, T const &attribute) {
			return *static_cast<T*>(set_particle_attribute_ptr(
				particle,
				&attribute
			));
		}

		bool remove_particle_attribute_ptr(particle_id particle, void *out_particle_attribute);

		template <typename T>
		bool remove_particle_attribute(particle_id particle, T &out_particle_attribute) {
			assert(typeid(T) == type_info && "error: attempted to access attribute of wrong type");
			return remove_particle_attribute_ptr(particle, &out_particle_attribute);
		}

		bool remove_particle_attribute(particle_id particle);
	};

	struct particle_storage {
	public:
		std::vector<attribute_storage> particles;
		std::unordered_map<std::type_index, attribute_id> attribute_map;

	public:
		particle_storage() noexcept;
		particle_storage(size_t attribute_capacity) noexcept;

	public:
		inline attribute_count_t attribute_count() const noexcept {
			return particles.size();
		}

		inline particle_count_t particle_count() const {
			return attribute_count() > 0 ? particles.at(0).particle_count() : 0;
		}

	private:
		template <typename T>
		bool get_or_emit_attribute(attribute_id& out_attribute) const {
			auto it = attribute_map.find(typeid(T));
			if (it != attribute_map.end()) {
				out_attribute = it->second;
				return true;
			} else {
				out_attribute = attribute_count();
				return false;
			}
		}
	
	public:
		template <typename T>
		bool particle_has_attribute(particle_id particle) const {
			attribute_id attribute = 0;
			if (!get_or_emit_attribute<T>(attribute)) {
				return false;
			}
			return particles.at(attribute).particle_has_attribute(particle);
		}


		template <typename T>
		T &get_particle_attribute(particle_id particle) {
			attribute_id attribute = 0;
			assert(get_or_emit_attribute<T>(attribute) && "error: attribute not found");
			return particles.at(attribute).get_particle_attribute<T>(particle);
		}

		template <typename T>
		T const &get_particle_attribute(particle_id particle) const {
			attribute_id attribute = 0;
			assert(get_or_emit_attribute<T>(attribute) && "error: attribute not found");
			return particles.at(attribute).get_particle_attribute<T>(particle);
		}

		template <typename T>
		T *get_particle_attribute_ptr(particle_id particle) {
			attribute_id attribute = 0;
			if (get_or_emit_attribute<T>(attribute) && particles.at(attribute).particle_has_attribute(particle)) {
				return static_cast<T *>(particles.at(attribute).get_particle_attribute_ptr(particle));
			} else {
				return nullptr;
			}
		}

		template <typename T>
		T const *get_particle_attribute_const_ptr(particle_id particle) const {
			attribute_id attribute = 0;
			if (get_or_emit_attribute<T>(attribute) && particles.at(attribute).particle_has_attribute(particle)) {
				return static_cast<T const *>(particles.at(attribute).get_particle_attribute_const_ptr(particle));
			} else {
				return nullptr;
			}
		}


		template <typename T>
		T &set_particle_attribute(particle_id particle, T &&attribute) {
			attribute_id attribute_id = 0;
			if (get_or_emit_attribute<T>(attribute_id)) {
				return particles.at(attribute_id).set_particle_attribute(particle, attribute);
			} else {
				attribute_map.insert({ typeid(T), attribute_id });
				//std::cout << "setting attribute: " << typeid(T).name() << std::endl;
				particles.emplace_back(attribute_storage::create_vector_storage<T>(particle_count()));
				return particles.at(attribute_id).set_particle_attribute(particle, attribute);
			}
		}

		
		template <typename T>
		bool remove_particle_attribute(particle_id particle, T &out_attribute) {
			attribute_id attribute_id = 0;
			if (!get_or_emit_attribute<T>(attribute_id)) {
				return false;
			}

			return particles.at(attribute_id).remove_particle_attribute<T>(particle, out_attribute);
		}

		particle_id clear_particle_attributes(particle_id particle) {
			for (attribute_storage &storage : particles) {
				if (storage.particle_has_attribute(particle)) {
					storage.remove_particle_attribute(particle);
				}
			}
			return particle;
		}

		particle_id copy_particle_onto(particle_id destination, particle_id source) {
			for (attribute_storage &storage : particles) {
				if (storage.particle_has_attribute(source)) {			
					storage.set_particle_attribute_ptr(
						destination,
						storage.get_particle_attribute_const_ptr(source)
					);
				}
			}
			return destination;
		}

		particle_id copy_particle(particle_id destination, particle_id source) {
			for (attribute_storage &storage : particles) {
				if (storage.particle_has_attribute(source)) {
					storage.set_particle_attribute_ptr(
						destination,
						storage.get_particle_attribute_const_ptr(source)
					);
				} else {
					storage.remove_particle_attribute(destination);
				}
			}
			return destination;
		}

	private:
		template <
			typename T,
			typename ...Attributes,
			typename Tuple = T::particle_deconstruct
		>
		std::tuple<Attributes& ...> set_particle_attributes_deconstruct_impl(
			particle_id particle,
			T const &attributes,
			std::tuple<Attributes ...>
		) {
			return { set_particle_attribute<Attributes>(
				particle,
				std::move(std::get<Attributes>(attributes.deconstruct()))
			)... };
		}

		template <
			typename T,
			typename ...Attributes,
			typename = typename std::enable_if_t<std::is_constructible_v<T, Attributes...>>
		>
		T get_particle_attributes_construct_impl(particle_id particle, std::tuple<Attributes ...>) {
			return T(get_particle_attribute<Attributes>(particle)...);
		}

	public:
		template <
			typename T,
			typename ...Attributes,
			typename Tuple = T::particle_deconstruct
		>
		auto set_particle_attributes_deconstruct(particle_id particle, T const &attributes) noexcept {
			return set_particle_attributes_deconstruct_impl<T>(
				particle,
				attributes, 
				Tuple{}
			);
		}

		template <
			typename T,
			typename Tuple = T::particle_deconstruct
		>
		T get_particle_attributes_construct(particle_id particle) {
			return get_particle_attributes_construct_impl<T>(particle, Tuple{});
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
		template <
			typename ...Ts,
			typename F = std::function<void(Ts& ...)>
		>	
		particle_count_t iter(F const &func) {
			particle_count_t count = 0;
			for (size_t i = 0; i < particle_count(); ++i) {
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

		template <
			typename ...Ts,
			typename F = std::function<void(particle_id, Ts& ...)>
		>	
		particle_count_t iter_indexed(F const &func) {
			particle_count_t count = 0;
			for (size_t i = 0; i < particle_count(); ++i) {
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
					call(func, std::tuple_cat(std::make_tuple(i), make_references(attributes)));
				}
			}

			return count;
		}
	};
}

#endif